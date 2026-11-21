/**
 * @file bc2_gatt_nimble.c
 * @brief NimBLE implementation of bc2_gatt_ops_t: scanner, single initiator, MTU exchange,
 *        discovery, read / write / subscribe and notification forwarding.
 *
 * Threading
 *   - The ops (scan_start, connect, read, ...) run in the core task / USB task, always under the
 *     core mutex, and only start NimBLE procedures. NimBLE API calls are made WITHOUT holding
 *     s_lock, so no lock-order problem with the NimBLE host mutex can arise.
 *   - All NimBLE callbacks run in the NimBLE host task. They update the link table under s_lock and
 *     queue events; they never call into bc2_core.
 *   - s_lock (a FreeRTOS mutex) protects the link table, the scan state, the filter and counters.
 *     It is only ever held for a few instructions.
 *
 * Events: state events (link up/down, completions, discovery) must not be lost; scan reports and
 * notifications are droppable when the event queue is more than half full (counted). If a state
 * event cannot be queued at all, s_state_lost is raised and the core drops every link (safety).
 */
#include "bc2_gatt_nimble.h"

#include <stdlib.h>
#include <string.h>

#include "bc2_scanfilter.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "sdkconfig.h"

/* ------------------------------------------------------------------------- */
/* Configuration                                                              */
/* ------------------------------------------------------------------------- */

/**
 * Links announced to the host: the smaller of the Kconfig value and the NimBLE connection slots.
 * The controller budget behind the default (CONFIG_BT_CTRL_BLE_MAX_ACT 10 - 5 advertising sets - 1 scan = 4)
 * is deliberately not enforced here, so the connection spike (docs/verification.md) can try 5 by raising
 * BC2ADV_MAX_CONNS, CONFIG_BT_NIMBLE_MAX_CONNECTIONS together.
 */
#define MAX_LINKS (CONFIG_BC2ADV_MAX_CONNS < CONFIG_BT_NIMBLE_MAX_CONNECTIONS ? CONFIG_BC2ADV_MAX_CONNS : CONFIG_BT_NIMBLE_MAX_CONNECTIONS)

/**
 * Adapter -> core events (about 10 KiB). One discovery posts up to ~50 events in a burst from the NimBLE host task
 * (higher priority than the core task); with LINK/OP_DONE events of other links a delayed core task could overflow
 * 64 slots. An overflow raises s_state_lost and triggers a safety stop of all links (safe, but harsh), so the queue
 * is sized with margin (see docs/development.md, "Event queue overflow").
 */
#define EVENT_QUEUE_LEN 128
/** Advertising sets that can run at once: NimBLE counts instances beyond instance 0. */
#define ADV_SETS (CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES + 1)
/** Log tag of this file. */
static const char *const TAG = "bc2_gatt_nimble";
#define MTU_WAIT_MS 2000                /**< a peer that does not answer the MTU request: continue with 23 */
#define SUPERVISION_TIMEOUT 400         /**< 4 s in 10 ms units */
#define INIT_SCAN_ITVL 96               /**< 60 ms: scan parameters of the initiator itself */
#define INIT_SCAN_WINDOW 48             /**< 30 ms */
#define MAX_SERVICES 24                 /**< reported per discovery */
#define MAX_CHARS 24                    /**< reported per service */
#define UUID_CCCD 0x2902                /**< Client Characteristic Configuration descriptor */

/* ------------------------------------------------------------------------- */
/* State                                                                      */
/* ------------------------------------------------------------------------- */

typedef enum { LINK_FREE, LINK_CONNECTING, LINK_MTU, LINK_UP } link_state_t;

/** One discovered service. */
typedef struct {
    uint16_t start, end;
    uint8_t uuid_len;
    uint8_t uuid[16];
} svc_t;

/** One discovered characteristic of the service that is being processed. */
typedef struct {
    uint16_t def_handle, value_handle, cccd_handle;
    uint8_t props;
    uint8_t uuid_len;
    uint8_t uuid[16];
} chr_t;

/** Discovery context, allocated by DISCOVER and freed when it ends or the link drops. */
typedef struct {
    svc_t svc[MAX_SERVICES];
    uint8_t n_svc;
    uint8_t cur_svc;                    /**< service whose characteristics are being discovered */
    chr_t chr[MAX_CHARS];
    uint8_t n_chr;
    uint8_t cur_chr;                    /**< characteristic whose descriptors are being discovered */
} disc_t;

/** One connection slot, indexed by the host's conn_id. */
typedef struct {
    link_state_t state;
    uint16_t handle;                    /**< NimBLE connection handle (valid from MTU state on) */
    int64_t mtu_deadline_us;            /**< end of the MTU wait */
    disc_t *disc;                       /**< running discovery or NULL */
} link_t;

static link_t s_link[BC2_GATT_MAX_CONNS_LIMIT];
static int s_attempt = -1;              /**< conn_id of the running connect attempt */
static bool s_scan_wanted;              /**< the host wants the scan to run */
static bool s_scan_paused;              /**< scan stopped by us for the duration of a connect attempt */
static bc2_req_scan_start_t s_scan_params;
static bc2_scanfilter_t s_filter;
static bc2_addrcache_t s_addr_cache;
static uint32_t s_drop_scan, s_drop_notify;   /**< dropped since the last stats() call */
static volatile bool s_state_lost;
static QueueHandle_t s_evq;
static SemaphoreHandle_t s_lock;
static void (*s_wake)(void);

/** Milliseconds since boot (wraps after 49 days; the filter and core are wrap safe). */
static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void lock(void) { xSemaphoreTake(s_lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_lock); }

/* ------------------------------------------------------------------------- */
/* Event queue                                                                */
/* ------------------------------------------------------------------------- */

/** Queue a state event (never dropped on purpose; failure raises s_state_lost) and wake the core. */
static void post_state(const bc2_gatt_event_t *ev)
{
    if (xQueueSend(s_evq, ev, 0) != pdTRUE) {
        s_state_lost = true;
    }
    if (s_wake != NULL) {
        s_wake();
    }
}

/** Queue a droppable event; false if the queue is more than half full (caller counts the drop). */
static bool post_droppable(const bc2_gatt_event_t *ev)
{
    if (uxQueueSpacesAvailable(s_evq) <= EVENT_QUEUE_LEN / 2) {
        return false;
    }
    if (xQueueSend(s_evq, ev, 0) != pdTRUE) {
        return false;
    }
    if (s_wake != NULL) {
        s_wake();
    }
    return true;
}

/** Fill an event header. */
static void ev_init(bc2_gatt_event_t *ev, bc2_gev_kind_t kind, uint8_t conn_id)
{
    memset(ev, 0, sizeof(*ev));
    ev->kind = (uint8_t)kind;
    ev->conn_id = conn_id;
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

/** NimBLE stores addresses little endian, the protocol uses display order. */
static void addr_to_display(const uint8_t nimble[6], uint8_t out[6])
{
    for (int i = 0; i < 6; i++) {
        out[i] = nimble[5 - i];
    }
}

/** HCI reason of a NimBLE status (0x200 + HCI code), stick specific value otherwise. */
static uint8_t reason_of(int status)
{
    if (status >= BLE_HS_ERR_HCI_BASE && status < BLE_HS_ERR_HCI_BASE + 0x100) {
        return (uint8_t)(status - BLE_HS_ERR_HCI_BASE);
    }
    if (status == BLE_HS_ETIMEOUT) {
        return BC2_REASON_CONNECT_TIMEOUT;
    }
    return BC2_REASON_INTERNAL;
}

/** Map a NimBLE return code of a starting call to a protocol status. */
static bc2_status_t map_start_rc(int rc, uint16_t *rc_out)
{
    *rc_out = (uint16_t)(rc & 0xFFFF);
    switch (rc) {
    case BLE_HS_EBUSY:
    case BLE_HS_ENOMEM:
    case BLE_HS_ENOMEM_EVT:
        return BC2_ERR_BUSY;             /* controller / mbuf pool momentarily full: retry later */
    case BLE_HS_ENOTCONN:
        return BC2_ERR_NOT_CONNECTED;
    default:
        return BC2_ERR_BLE_ERROR;
    }
}

/** Fill status/code of an OP_DONE event from a GATT callback error. */
static void map_gatt_error(const struct ble_gatt_error *error, bc2_gatt_event_t *ev)
{
    const int status = error->status;
    if (status == 0) {
        ev->status = BC2_OK;
    } else if (status > BLE_HS_ERR_ATT_BASE && status < BLE_HS_ERR_ATT_BASE + 0x100) {
        ev->status = BC2_ERR_ATT_ERROR;
        ev->code = (uint16_t)(status - BLE_HS_ERR_ATT_BASE);
    } else if (status == BLE_HS_ENOTCONN) {
        ev->status = BC2_ERR_DISCONNECTED;
    } else {
        ev->status = BC2_ERR_BLE_ERROR;
        ev->code = (uint16_t)(status & 0xFFFF);
    }
}

/** Copy a UUID into 2 or 16 little endian bytes; returns the length. */
static uint8_t copy_uuid(const ble_uuid_any_t *uuid, uint8_t out[16])
{
    if (uuid->u.type == BLE_UUID_TYPE_16) {
        out[0] = (uint8_t)(uuid->u16.value & 0xFFu);
        out[1] = (uint8_t)(uuid->u16.value >> 8);
        return 2;
    }
    memcpy(out, uuid->u128.value, 16);       /* NimBLE keeps 128 bit UUIDs in ATT wire order */
    return 16;
}

/* ------------------------------------------------------------------------- */
/* Scanning                                                                   */
/* ------------------------------------------------------------------------- */

static int scan_gap_event(struct ble_gap_event *event, void *arg);

/** Start the controller scan with the stored parameters. */
static int scan_start_controller(void)
{
    struct ble_gap_ext_disc_params params;
    memset(&params, 0, sizeof(params));
    params.itvl = s_scan_params.interval;
    params.window = s_scan_params.window;
    params.passive = (s_scan_params.flags & BC2_SCAN_FLAG_ACTIVE) ? 0 : 1;
    /* duration 0 / period 0: scan until stopped; duplicates are filtered by bc2_scanfilter instead of the
     * controller (the controller would hide changed data of a known advertiser). */
    return ble_gap_ext_disc(BLE_OWN_ADDR_PUBLIC, 0, 0, 0, 0, 0, &params, NULL, scan_gap_event, NULL);
}

/** Callback of the scan: forwards legacy advertising PDUs that pass the filter. */
static int scan_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        lock();
        const bool unexpected = s_scan_wanted && !s_scan_paused;    /* not our own stop / pause */
        if (unexpected) {
            s_scan_wanted = false;
        }
        unlock();
        if (unexpected) {
            bc2_gatt_event_t ev;
            ev_init(&ev, BC2_GEV_SCAN_STOPPED, 0);
            ev.code = (uint16_t)(event->disc_complete.reason & 0xFFFF);
            post_state(&ev);
        }
        return 0;
    }
    if (event->type != BLE_GAP_EVENT_EXT_DISC) {
        return 0;
    }
    const struct ble_gap_ext_disc_desc *d = &event->ext_disc;
    if ((d->props & BLE_HCI_ADV_LEGACY_MASK) == 0) {
        return 0;                                  /* extended advertising: no BC2 device uses it */
    }
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_SCAN_REPORT, 0);
    addr_to_display(d->addr.val, ev.addr);
    ev.addr_type = (uint8_t)(d->addr.type & 1u);   /* public / random (identity types map onto them) */
    ev.rssi = d->rssi;
    ev.len = d->length_data > BC2_GATT_MAX_SCAN_DATA ? (uint8_t)BC2_GATT_MAX_SCAN_DATA : d->length_data;
    memcpy(ev.data, d->data, ev.len);
    const bool scan_rsp = (d->props & BLE_HCI_ADV_SCAN_RSP_MASK) != 0;
    ev.flags = (uint8_t)(((d->props & BLE_HCI_ADV_CONN_MASK) ? BC2_REPORT_CONNECTABLE : 0u) |
                         (scan_rsp ? BC2_REPORT_SCAN_RSP : 0u) |
                         ((d->props & BLE_HCI_ADV_DIRECT_MASK) ? BC2_REPORT_DIRECTED : 0u));

    lock();
    bc2_addrcache_put(&s_addr_cache, ev.addr, ev.addr_type);   /* CONNECT auto resolves types from here */
    const bc2_sf_verdict_t v = bc2_scanfilter_check(&s_filter, ev.addr, ev.addr_type, scan_rsp, ev.data, ev.len, now_ms());
    if (v == BC2_SF_OVER_BUDGET) {
        s_drop_scan++;
    }
    unlock();
    if (v == BC2_SF_REPORT && !post_droppable(&ev)) {
        lock();
        s_drop_scan++;                             /* event queue pressure */
        unlock();
    }
    return 0;
}

static bc2_status_t op_scan_start(void *ctx, const bc2_req_scan_start_t *p, uint16_t *rc_out)
{
    (void)ctx;
    lock();
    s_scan_params = *p;
    bc2_scanfilter_init(&s_filter, p->refresh_ms, (p->flags & BC2_SCAN_FLAG_DUPLICATES) != 0);
    s_scan_wanted = true;
    s_scan_paused = false;
    unlock();
    const int rc = scan_start_controller();
    if (rc != 0) {
        lock();
        s_scan_wanted = false;
        unlock();
        *rc_out = (uint16_t)(rc & 0xFFFF);
        return rc == BLE_HS_ENOMEM ? BC2_ERR_NO_RESOURCES : (rc == BLE_HS_EBUSY ? BC2_ERR_BUSY : BC2_ERR_BLE_ERROR);
    }
    return BC2_OK;
}

static void op_scan_stop(void *ctx)
{
    (void)ctx;
    lock();
    const bool was_running = s_scan_wanted && !s_scan_paused;
    s_scan_wanted = false;                         /* first: the completion callback must not report this stop */
    s_scan_paused = false;
    unlock();
    if (was_running) {
        ble_gap_disc_cancel();
    }
}

/** After a connect attempt: restart the scan if it was paused for the attempt. */
static void resume_scan_if_paused(void)
{
    lock();
    const bool resume = s_scan_wanted && s_scan_paused;
    s_scan_paused = false;
    unlock();
    if (resume && scan_start_controller() != 0) {
        lock();
        s_scan_wanted = false;
        unlock();
        bc2_gatt_event_t ev;
        ev_init(&ev, BC2_GEV_SCAN_STOPPED, 0);
        post_state(&ev);
    }
}

/* ------------------------------------------------------------------------- */
/* Connecting                                                                 */
/* ------------------------------------------------------------------------- */

static int central_gap_event(struct ble_gap_event *event, void *arg);

/** Link is usable: report it with the final MTU and interval. Called once per link (state MTU -> UP). */
static void finish_link_up(uint8_t id, uint16_t handle, bool mtu_ok)
{
    lock();
    const bool ours = s_link[id].state == LINK_MTU && s_link[id].handle == handle;
    if (ours) {
        s_link[id].state = LINK_UP;
    }
    unlock();
    if (!ours) {
        return;                                    /* link already gone, or already reported */
    }
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_LINK_UP, id);
    uint16_t mtu = mtu_ok ? ble_att_mtu(handle) : BC2_GATT_DEFAULT_MTU;
    ev.u16a = mtu < BC2_GATT_DEFAULT_MTU ? BC2_GATT_DEFAULT_MTU : mtu;
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(handle, &desc) == 0) {
        ev.u16b = desc.conn_itvl;
        addr_to_display(desc.peer_id_addr.val, ev.addr);
    }
    post_state(&ev);
}

/** MTU exchange finished (an ATT error of an old peer counts as "not supported", never as failure). */
static int mtu_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg)
{
    (void)mtu;
    finish_link_up((uint8_t)(uintptr_t)arg, conn_handle, error->status == 0);
    return 0;
}

/** CONNECT event of a central attempt. */
static void on_connect_event(uint8_t id, const struct ble_gap_event *event)
{
    if (event->connect.status != 0) {              /* attempt ended without a link */
        lock();
        s_link[id].state = LINK_FREE;
        if (s_attempt == id) {
            s_attempt = -1;
        }
        unlock();
        bc2_gatt_event_t ev;
        ev_init(&ev, BC2_GEV_CONNECT_FAILED, id);
        ev.code = reason_of(event->connect.status);
        post_state(&ev);
        resume_scan_if_paused();
        return;
    }
    const uint16_t handle = event->connect.conn_handle;
    lock();
    s_link[id].state = LINK_MTU;
    s_link[id].handle = handle;
    s_link[id].mtu_deadline_us = esp_timer_get_time() + (int64_t)MTU_WAIT_MS * 1000;
    if (s_attempt == id) {
        s_attempt = -1;                            /* the initiator is free again */
    }
    unlock();
    resume_scan_if_paused();
    /* The stick negotiates the MTU itself: BC2 never asks for one. Failure is not fatal. */
    if (ble_gattc_exchange_mtu(handle, mtu_cb, (void *)(uintptr_t)id) != 0) {
        finish_link_up(id, handle, false);
    }
}

/** Free the discovery context of a link (caller must NOT hold s_lock). */
static void drop_discovery(uint8_t id)
{
    lock();
    disc_t *d = s_link[id].disc;
    s_link[id].disc = NULL;
    unlock();
    free(d);
}

/** DISCONNECT event of a central link. */
static void on_disconnect_event(uint8_t id, const struct ble_gap_event *event)
{
    lock();
    const bool known = s_link[id].state == LINK_MTU || s_link[id].state == LINK_UP;
    s_link[id].state = LINK_FREE;
    unlock();
    drop_discovery(id);
    if (!known) {
        return;
    }
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_LINK_DOWN, id);
    ev.code = reason_of(event->disconnect.reason);
    post_state(&ev);
}

/** Copy a received notification into an event (flags: bit0 indication, bit1 truncated). */
static void on_notify_event(uint8_t id, const struct ble_gap_event *event)
{
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_NOTIFY, id);
    ev.u16a = event->notify_rx.attr_handle;
    const uint16_t total = OS_MBUF_PKTLEN(event->notify_rx.om);
    ev.len = total > BC2_GATT_MAX_VALUE ? (uint8_t)BC2_GATT_MAX_VALUE : (uint8_t)total;
    os_mbuf_copydata(event->notify_rx.om, 0, ev.len, ev.data);
    ev.flags = (uint8_t)((event->notify_rx.indication ? BC2_VALUE_INDICATION : 0u) |
                         (total > BC2_GATT_MAX_VALUE ? BC2_VALUE_TRUNCATED : 0u));
    if (!post_droppable(&ev)) {
        lock();
        s_drop_notify++;
        unlock();
    }
}

/** GAP callback of every central connection (arg = conn_id). */
static int central_gap_event(struct ble_gap_event *event, void *arg)
{
    const uint8_t id = (uint8_t)(uintptr_t)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        on_connect_event(id, event);
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        on_disconnect_event(id, event);
        break;
    case BLE_GAP_EVENT_NOTIFY_RX:
        on_notify_event(id, event);
        break;
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        return 0;                                  /* accept the peer's requested parameters */
    default:
        break;
    }
    return 0;
}

static bc2_status_t op_connect(void *ctx, uint8_t id, const uint8_t addr[6], uint8_t addr_type,
                               uint16_t itvl_min, uint16_t itvl_max, uint32_t duration_ms, uint16_t *rc_out)
{
    (void)ctx;
    lock();
    if (s_attempt >= 0 || s_link[id].state != LINK_FREE) {
        unlock();
        return BC2_ERR_BUSY;                       /* the controller has one initiator */
    }
    if (addr_type == BC2_ADDR_TYPE_AUTO && !bc2_addrcache_get(&s_addr_cache, addr, &addr_type)) {
        addr_type = 0;                             /* never seen while scanning: assume public */
    }
    s_attempt = id;
    s_link[id].state = LINK_CONNECTING;
    unlock();

    ble_addr_t peer = {.type = addr_type == 0 ? BLE_ADDR_PUBLIC : BLE_ADDR_RANDOM};
    for (int i = 0; i < 6; i++) {
        peer.val[i] = addr[5 - i];
    }
    struct ble_gap_conn_params params;
    memset(&params, 0, sizeof(params));
    params.scan_itvl = INIT_SCAN_ITVL;
    params.scan_window = INIT_SCAN_WINDOW;
    params.itvl_min = itvl_min;
    params.itvl_max = itvl_max;
    params.latency = 0;
    params.supervision_timeout = SUPERVISION_TIMEOUT;
    const int32_t duration = duration_ms == 0 ? BLE_HS_FOREVER : (int32_t)duration_ms;

    int rc = ble_gap_ext_connect(BLE_OWN_ADDR_PUBLIC, &peer, duration, BLE_GAP_LE_PHY_1M_MASK, &params, NULL,
                                 NULL, central_gap_event, (void *)(uintptr_t)id);
    if (rc != 0) {
        /* Some controller states refuse to initiate while scanning: pause the scan for the attempt
         * and resume it in on_connect_event(). */
        lock();
        const bool scanning = s_scan_wanted && !s_scan_paused;
        if (scanning) {
            s_scan_paused = true;
        }
        unlock();
        if (scanning) {
            ble_gap_disc_cancel();
            rc = ble_gap_ext_connect(BLE_OWN_ADDR_PUBLIC, &peer, duration, BLE_GAP_LE_PHY_1M_MASK, &params, NULL,
                                     NULL, central_gap_event, (void *)(uintptr_t)id);
            if (rc != 0) {
                resume_scan_if_paused();
            }
        }
    }
    if (rc != 0) {
        lock();
        s_attempt = -1;
        s_link[id].state = LINK_FREE;
        unlock();
        *rc_out = (uint16_t)(rc & 0xFFFF);
        return rc == BLE_HS_ENOMEM ? BC2_ERR_NO_RESOURCES : (rc == BLE_HS_EALREADY ? BC2_ERR_BUSY : BC2_ERR_BLE_ERROR);
    }
    return BC2_OK;
}

static void op_connect_cancel(void *ctx)
{
    (void)ctx;
    /* The outcome arrives as a CONNECT event with a failure status (or the link came up first). If no
     * attempt is running any more, the core's 2 s cancel guard ends the wait. */
    ble_gap_conn_cancel();
}

static bool op_terminate(void *ctx, uint8_t id)
{
    (void)ctx;
    lock();
    const link_state_t state = s_link[id].state;
    const uint16_t handle = s_link[id].handle;
    unlock();
    if (state == LINK_CONNECTING) {
        return ble_gap_conn_cancel() == 0;
    }
    if (state != LINK_MTU && state != LINK_UP) {
        return true;                               /* nothing to terminate */
    }
    const int rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
    return rc == 0 || rc == BLE_HS_EALREADY || rc == BLE_HS_ENOTCONN;   /* already on its way down */
}

/* ------------------------------------------------------------------------- */
/* Discovery: services, then characteristics, then CCCD descriptors           */
/* ------------------------------------------------------------------------- */

static void next_service(uint8_t id, uint16_t handle);

/** Finish the discovery of a link with @p status and free its context. */
static void discovery_done(uint8_t id, bc2_status_t status)
{
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_DISCOVER_DONE, id);
    ev.status = (uint8_t)status;
    drop_discovery(id);
    post_state(&ev);
}

/** Get the discovery context of a link if @p handle still names it (callbacks can arrive late). */
static disc_t *disc_of(uint8_t id, uint16_t handle)
{
    lock();
    disc_t *d = (s_link[id].state == LINK_UP && s_link[id].handle == handle) ? s_link[id].disc : NULL;
    unlock();
    return d;
}

/** Emit the EVT_GATT_CHAR events of the current service and continue with the next service. */
static void flush_chars_and_continue(uint8_t id, uint16_t handle, disc_t *d)
{
    for (uint8_t i = 0; i < d->n_chr; i++) {
        bc2_gatt_event_t ev;
        ev_init(&ev, BC2_GEV_CHAR, id);
        ev.u16a = d->chr[i].value_handle;
        ev.u16b = d->chr[i].cccd_handle;
        ev.code = d->chr[i].props;
        ev.len = d->chr[i].uuid_len;
        memcpy(ev.data, d->chr[i].uuid, ev.len);
        post_state(&ev);
    }
    d->n_chr = 0;
    d->cur_svc++;
    next_service(id, handle);
}

/** Descriptor discovery callback: remember the CCCD handle of the characteristic being processed. */
static int dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                  const struct ble_gatt_dsc *dsc, void *arg);

/** Start the descriptor discovery of the next notify/indicate characteristic, or finish the service. */
static void next_descriptor(uint8_t id, uint16_t handle, disc_t *d)
{
    while (d->cur_chr < d->n_chr) {
        chr_t *c = &d->chr[d->cur_chr];
        if ((c->props & 0x30u) == 0) {             /* no notify / indicate: no CCCD to look for */
            d->cur_chr++;
            continue;
        }
        /* descriptors live between the value handle and the next characteristic definition (or the service end) */
        const chr_t *next = d->cur_chr + 1u < d->n_chr ? &d->chr[d->cur_chr + 1] : NULL;
        uint16_t first, last;
        /* NimBLE's "start_handle" is the LAST KNOWN handle, not the first one to scan: it sends its Find
         * Information request from start_handle + 1. Passing value_handle + 1 skipped the CCCD at
         * value_handle + 1 (the usual place) and the CCCD was never found. Pass the value handle itself. */
        if (!bc2_gatt_descriptor_range(c->value_handle, next != NULL, next != NULL ? next->def_handle : 0,
                                       d->svc[d->cur_svc].end, &first, &last) ||
            ble_gattc_disc_all_dscs(handle, (uint16_t)(first - 1u), last, dsc_cb,
                                    (void *)(uintptr_t)id) != 0) {
            d->cur_chr++;                          /* nothing to scan, or the stack refused: report without CCCD */
            continue;
        }
        return;                                    /* dsc_cb continues */
    }
    flush_chars_and_continue(id, handle, d);
}

static int dsc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                  const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle;
    const uint8_t id = (uint8_t)(uintptr_t)arg;
    disc_t *d = disc_of(id, conn_handle);
    if (d == NULL) {
        return 0;
    }
    if (error->status == 0) {
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16 && dsc->uuid.u16.value == UUID_CCCD) {
            d->chr[d->cur_chr].cccd_handle = dsc->handle;
        }
    } else {                                       /* BLE_HS_EDONE or an error: either way this characteristic is done */
        d->cur_chr++;
        next_descriptor(id, conn_handle, d);
    }
    return 0;
}

/** Characteristic discovery callback of the current service. */
static int chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg)
{
    const uint8_t id = (uint8_t)(uintptr_t)arg;
    disc_t *d = disc_of(id, conn_handle);
    if (d == NULL) {
        return 0;
    }
    if (error->status == 0) {
        if (d->n_chr < MAX_CHARS) {
            chr_t *c = &d->chr[d->n_chr++];
            memset(c, 0, sizeof(*c));
            c->def_handle = chr->def_handle;
            c->value_handle = chr->val_handle;
            c->props = chr->properties;
            c->uuid_len = copy_uuid(&chr->uuid, c->uuid);
        }
    } else if (error->status == BLE_HS_EDONE) {
        d->cur_chr = 0;
        next_descriptor(id, conn_handle, d);
    } else {
        discovery_done(id, BC2_ERR_BLE_ERROR);
    }
    return 0;
}

/** Discover the characteristics of the next service, or finish when all services are done. */
static void next_service(uint8_t id, uint16_t handle)
{
    disc_t *d = disc_of(id, handle);
    if (d == NULL) {
        return;
    }
    if (d->cur_svc >= d->n_svc) {
        discovery_done(id, BC2_OK);
        return;
    }
    const svc_t *s = &d->svc[d->cur_svc];
    if (ble_gattc_disc_all_chrs(handle, s->start, s->end, chr_cb, (void *)(uintptr_t)id) != 0) {
        discovery_done(id, BC2_ERR_BLE_ERROR);
    }
}

/** Service discovery callback: collects services, then reports them all and starts the characteristic phase. */
static int svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *arg)
{
    const uint8_t id = (uint8_t)(uintptr_t)arg;
    disc_t *d = disc_of(id, conn_handle);
    if (d == NULL) {
        return 0;
    }
    if (error->status == 0) {
        if (d->n_svc < MAX_SERVICES) {
            svc_t *s = &d->svc[d->n_svc++];
            s->start = svc->start_handle;
            s->end = svc->end_handle;
            s->uuid_len = copy_uuid(&svc->uuid, s->uuid);
        }
    } else if (error->status == BLE_HS_EDONE) {
        for (uint8_t i = 0; i < d->n_svc; i++) {   /* all services first, as documented in protocol 16 */
            bc2_gatt_event_t ev;
            ev_init(&ev, BC2_GEV_SERVICE, id);
            ev.u16a = d->svc[i].start;
            ev.u16b = d->svc[i].end;
            ev.len = d->svc[i].uuid_len;
            memcpy(ev.data, d->svc[i].uuid, ev.len);
            post_state(&ev);
        }
        next_service(id, conn_handle);
    } else {
        discovery_done(id, BC2_ERR_BLE_ERROR);
    }
    return 0;
}

static bc2_status_t op_discover(void *ctx, uint8_t id, uint16_t *rc_out)
{
    (void)ctx;
    disc_t *d = calloc(1, sizeof(*d));
    if (d == NULL) {
        return BC2_ERR_NO_RESOURCES;
    }
    lock();
    const bool up = s_link[id].state == LINK_UP;
    const uint16_t handle = s_link[id].handle;
    if (up) {
        s_link[id].disc = d;
    }
    unlock();
    if (!up) {
        free(d);
        return BC2_ERR_NOT_CONNECTED;
    }
    const int rc = ble_gattc_disc_all_svcs(handle, svc_cb, (void *)(uintptr_t)id);
    if (rc != 0) {
        lock();
        const bool still_ours = s_link[id].disc == d;     /* the link may have dropped meanwhile and freed it */
        if (still_ours) {
            s_link[id].disc = NULL;
        }
        unlock();
        if (still_ours) {
            free(d);
        }
        return map_start_rc(rc, rc_out);
    }
    return BC2_OK;
}

/* ------------------------------------------------------------------------- */
/* Read, write, subscribe                                                     */
/* ------------------------------------------------------------------------- */

/** Handle of an UP link, or false. */
static bool link_handle(uint8_t id, uint16_t *handle)
{
    lock();
    const bool up = s_link[id].state == LINK_UP;
    *handle = s_link[id].handle;
    unlock();
    return up;
}

/** Completion of READ: value (truncated to 56 bytes) or error. */
static int read_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_OP_DONE, (uint8_t)(uintptr_t)arg);
    map_gatt_error(error, &ev);
    if (error->status == 0 && attr != NULL && attr->om != NULL) {
        const uint16_t total = OS_MBUF_PKTLEN(attr->om);
        ev.len = total > BC2_GATT_MAX_VALUE ? (uint8_t)BC2_GATT_MAX_VALUE : (uint8_t)total;
        os_mbuf_copydata(attr->om, 0, ev.len, ev.data);
        ev.flags = total > BC2_GATT_MAX_VALUE ? BC2_READ_TRUNCATED : 0u;
    }
    post_state(&ev);
    return 0;
}

/** Completion of WRITE and SUBSCRIBE (CCCD write). */
static int write_cb(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    bc2_gatt_event_t ev;
    ev_init(&ev, BC2_GEV_OP_DONE, (uint8_t)(uintptr_t)arg);
    map_gatt_error(error, &ev);
    post_state(&ev);
    return 0;
}

static bc2_status_t op_read(void *ctx, uint8_t id, uint16_t attr, uint16_t *rc_out)
{
    (void)ctx;
    uint16_t handle;
    if (!link_handle(id, &handle)) {
        return BC2_ERR_NOT_CONNECTED;
    }
    const int rc = ble_gattc_read(handle, attr, read_cb, (void *)(uintptr_t)id);
    return rc == 0 ? BC2_OK : map_start_rc(rc, rc_out);
}

static bc2_status_t op_write(void *ctx, uint8_t id, uint16_t attr, const uint8_t *data, uint8_t len, uint16_t *rc_out)
{
    (void)ctx;
    uint16_t handle;
    if (!link_handle(id, &handle)) {
        return BC2_ERR_NOT_CONNECTED;
    }
    const int rc = ble_gattc_write_flat(handle, attr, data, len, write_cb, (void *)(uintptr_t)id);
    return rc == 0 ? BC2_OK : map_start_rc(rc, rc_out);
}

static bc2_status_t op_write_cmd(void *ctx, uint8_t id, uint16_t attr, const uint8_t *data, uint8_t len,
                                 uint16_t *rc_out)
{
    (void)ctx;
    uint16_t handle;
    if (!link_handle(id, &handle)) {
        return BC2_ERR_NOT_CONNECTED;
    }
    const int rc = ble_gattc_write_no_rsp_flat(handle, attr, data, len);
    return rc == 0 ? BC2_OK : map_start_rc(rc, rc_out);
}

static bc2_status_t op_subscribe(void *ctx, uint8_t id, uint16_t cccd, uint8_t mode, uint16_t *rc_out)
{
    (void)ctx;
    uint16_t handle;
    if (!link_handle(id, &handle)) {
        return BC2_ERR_NOT_CONNECTED;
    }
    /* CCCD value: bit0 notifications, bit1 indications. Notification handling is always active in
     * NimBLE (BLE_GAP_EVENT_NOTIFY_RX), so nothing is lost between the write and its completion. */
    const uint8_t value[2] = {mode == BC2_SUB_NOTIFY ? 0x01u : (mode == BC2_SUB_INDICATE ? 0x02u : 0x00u), 0x00};
    const int rc = ble_gattc_write_flat(handle, cccd, value, sizeof(value), write_cb, (void *)(uintptr_t)id);
    return rc == 0 ? BC2_OK : map_start_rc(rc, rc_out);
}

/* ------------------------------------------------------------------------- */
/* Events, statistics, initialisation                                         */
/* ------------------------------------------------------------------------- */

/** Links whose peer never answered the MTU request continue with the default MTU (checked every 20 ms at most). */
static void check_mtu_timeouts(void)
{
    static int64_t last_check_us;
    const int64_t now = esp_timer_get_time();
    if (now - last_check_us < 20000) {
        return;                                    /* poll_event() runs in a loop: do not scan the table every time */
    }
    last_check_us = now;
    for (uint8_t id = 0; id < BC2_GATT_MAX_CONNS_LIMIT; id++) {
        lock();
        const bool expired = s_link[id].state == LINK_MTU && now >= s_link[id].mtu_deadline_us;
        const uint16_t handle = s_link[id].handle;
        unlock();
        if (expired) {
            finish_link_up(id, handle, false);
        }
    }
}

static bool op_poll_event(void *ctx, bc2_gatt_event_t *ev)
{
    (void)ctx;
    check_mtu_timeouts();
    return xQueueReceive(s_evq, ev, 0) == pdTRUE;
}

/** Saturating u16 of a counter. */
static uint16_t sat16(uint32_t v)
{
    return v > 0xFFFFu ? 0xFFFFu : (uint16_t)v;
}

static void op_stats(void *ctx, bc2_gatt_adapter_stats_t *out)
{
    (void)ctx;
    lock();
    out->dropped_scan = sat16(s_drop_scan);
    out->dropped_notify = sat16(s_drop_notify);
    s_drop_scan = 0;
    s_drop_notify = 0;
    unlock();
    out->state_lost = s_state_lost;
    s_state_lost = false;
    out->free_heap = esp_get_free_heap_size();
    out->min_free_heap = esp_get_minimum_free_heap_size();
}

static uint8_t op_max_conns(void *ctx)
{
    (void)ctx;
    return MAX_LINKS;
}

esp_err_t bc2_gatt_nimble_init(void (*wake)(void))
{
    s_lock = xSemaphoreCreateMutex();
    s_evq = xQueueCreate(EVENT_QUEUE_LEN, sizeof(bc2_gatt_event_t));
    if (s_lock == NULL || s_evq == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_wake = wake;
    /* Controller budget check: advertising sets and the scan share CONFIG_BT_CTRL_BLE_MAX_ACT with the links. */
    if (CONFIG_BT_CTRL_BLE_MAX_ACT - ADV_SETS - 1 < MAX_LINKS) {
        ESP_LOGW(TAG, "controller budget %d - %d sets - 1 scan < %d links: CONNECT may fail with NO_RESOURCES",
                 CONFIG_BT_CTRL_BLE_MAX_ACT, ADV_SETS, MAX_LINKS);
    }
    bc2_addrcache_init(&s_addr_cache);
    bc2_scanfilter_init(&s_filter, BC2_SCAN_DEFAULT_REFRESH_MS, false);
    /* Our request in the MTU exchange; 64 covers the largest BC2 write (BuWizz 3: 38 bytes + 3). */
    return ble_att_set_preferred_mtu(CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU) == 0 ? ESP_OK : ESP_FAIL;
}

const bc2_gatt_ops_t *bc2_gatt_nimble_ops(void)
{
    static const bc2_gatt_ops_t ops = {
        .ctx = NULL,
        .max_conns = op_max_conns,
        .scan_start = op_scan_start,
        .scan_stop = op_scan_stop,
        .connect = op_connect,
        .connect_cancel = op_connect_cancel,
        .terminate = op_terminate,
        .discover = op_discover,
        .read = op_read,
        .write = op_write,
        .write_cmd = op_write_cmd,
        .subscribe = op_subscribe,
        .poll_event = op_poll_event,
        .stats = op_stats,
    };
    return &ops;
}
