/**
 * @file bc2_ble.c
 * @brief NimBLE extended-advertising adapter used with legacy PDUs only (see docs/protocol.md).
 *
 * Every protocol set maps to one NimBLE instance with legacy_pdu = 1, so only 31 byte legacy
 * PDUs on channels 37/38/39 appear on air. Updates change the data of the running instance in
 * place (legal for legacy PDUs), which keeps the latency to a few milliseconds.
 *
 * Threading: bc2_core calls the ops from the USB/core task, NimBLE calls gap_event() from the
 * host task. Shared state is guarded by a short critical section that is never held while
 * calling into NimBLE, so no lock-order problems with the host mutex can arise.
 */
#include "bc2_ble.h"

#include <string.h>

#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "sdkconfig.h"

/** Number of usable instances: NimBLE has 1 + CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES. */
#define NIMBLE_INSTANCES (CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES + 1)
/** Sets offered to the host: Kconfig limit capped by the compiled-in instance count. */
#define MAX_SETS ((CONFIG_BC2ADV_MAX_SETS) < (NIMBLE_INSTANCES) ? (CONFIG_BC2ADV_MAX_SETS) : (NIMBLE_INSTANCES))
/** rc value reported when the host is not synced yet. */
#define RC_NOT_READY 0xFFFFu

/** Per-instance state shared between the core task and the NimBLE host task. */
typedef struct {
    bool active;          /**< set is supposed to advertise (a restart is wanted when it stops) */
    bool needs_restart;   /**< restart failed while a connection was still open, retry on disconnect */
    uint8_t flags;        /**< SET_START flags (AD flags bit, PDU type) */
} inst_t;

/** Asynchronous failure of a set, collected for bc2_core_tick(). */
typedef struct {
    bool pending;
    uint8_t reason;
} failure_t;

static inst_t s_inst[MAX_SETS];
static failure_t s_fail[MAX_SETS];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;   /* guards s_inst / s_fail / counters */
static volatile bool s_synced;                               /* host synced with the controller */
static int s_connections;                                    /* currently open (to be terminated) links */
static uint8_t s_mac[6];                                     /* Bluetooth MAC, display order */
static uint32_t s_fallback_updates;                          /* debug: updates needing stop/set/start */

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

/** Record an asynchronous failure and mark the set inactive. */
static void report_failure(uint8_t id, uint8_t reason)
{
    portENTER_CRITICAL(&s_mux);
    s_inst[id].active = false;
    s_inst[id].needs_restart = false;
    s_fail[id].pending = true;
    s_fail[id].reason = reason;
    portEXIT_CRITICAL(&s_mux);
}

/** Copy the advertising bytes into a fresh mbuf (ownership passes to the NimBLE call). */
static struct os_mbuf *make_mbuf(const uint8_t *bytes, size_t len)
{
    struct os_mbuf *m = os_msys_get_pkthdr((uint16_t)len, 0);
    if (m != NULL && len > 0 && os_mbuf_append(m, bytes, (uint16_t)len) != 0) {
        os_mbuf_free_chain(m);
        return NULL;
    }
    return m;
}

/** Distinct static random address per instance, derived from the MAC (top two bits set). */
static void make_address(uint8_t id, ble_addr_t *addr)
{
    addr->type = BLE_ADDR_RANDOM;
    for (int i = 0; i < 6; i++) {
        addr->val[i] = s_mac[5 - i];             /* NimBLE stores addresses little endian */
    }
    addr->val[0] = (uint8_t)(addr->val[0] ^ (id + 1u));   /* last byte differs per instance */
    addr->val[5] |= 0xC0;                         /* static random address marker */
}

/** Try to (re)start an instance after its advertising ended (connection attempt). */
static void restart_instance(uint8_t id)
{
    bool active;
    portENTER_CRITICAL(&s_mux);
    active = s_inst[id].active;
    portEXIT_CRITICAL(&s_mux);
    if (!active) {
        return;                                   /* stopped by the host in the meantime */
    }
    const int rc = ble_gap_ext_adv_start(id, 0, 0);
    if (rc == 0 || rc == BLE_HS_EALREADY) {
        return;
    }
    bool connected;
    portENTER_CRITICAL(&s_mux);
    connected = s_connections > 0;
    if (connected) {
        s_inst[id].needs_restart = true;          /* the open link may block us, retry on disconnect */
    }
    portEXIT_CRITICAL(&s_mux);
    if (!connected) {
        report_failure(id, BC2_STOP_BLE_ERROR);
    }
}

/** Retry all restarts that were postponed because of an open connection. */
static void retry_pending_restarts(void)
{
    for (uint8_t id = 0; id < MAX_SETS; id++) {
        bool retry;
        portENTER_CRITICAL(&s_mux);
        retry = s_inst[id].active && s_inst[id].needs_restart;
        s_inst[id].needs_restart = false;
        portEXIT_CRITICAL(&s_mux);
        if (retry) {
            const int rc = ble_gap_ext_adv_start(id, 0, 0);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                report_failure(id, BC2_STOP_DISCONNECT_FAILED);
            }
        }
    }
}

/**
 * GAP callback of every instance (arg = instance id). The stick does not offer any service:
 * a connection attempt on the connectable ADV_IND is terminated immediately and advertising
 * continues.
 */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    const uint8_t id = (uint8_t)(uintptr_t)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            portENTER_CRITICAL(&s_mux);
            s_connections++;
            portEXIT_CRITICAL(&s_mux);
            /* If the terminate request fails the link stays open and the connectable set stays
             * down: tell the host instead of failing silently. */
            if (ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM) != 0) {
                report_failure(id, BC2_STOP_DISCONNECT_FAILED);
            }
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        portENTER_CRITICAL(&s_mux);
        if (s_connections > 0) {
            s_connections--;
        }
        portEXIT_CRITICAL(&s_mux);
        retry_pending_restarts();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        restart_instance(id);                     /* advertising ended because a peer connected */
        break;
    default:
        break;
    }
    return 0;
}

/** Set the (empty) scan response that the controller requires for scannable legacy sets. */
static int set_empty_scan_response(uint8_t id)
{
    return ble_gap_ext_adv_rsp_set_data(id, make_mbuf(NULL, 0));
}

/** Convert a NimBLE return code to the 16 bit value reported to the host. */
static uint16_t rc16(int rc)
{
    return (uint16_t)(rc & 0xFFFF);
}

/* ------------------------------------------------------------------------- */
/* bc2_ble_ops_t implementation                                               */
/* ------------------------------------------------------------------------- */

static bool op_ready(void *ctx) { (void)ctx; return s_synced; }

static uint8_t op_max_sets(void *ctx) { (void)ctx; return MAX_SETS; }

static void op_device_id(void *ctx, uint8_t out[6]) { (void)ctx; memcpy(out, s_mac, 6); }

/** Build the NimBLE parameters of a set from the protocol parameters. */
static struct ble_gap_ext_adv_params make_params(const bc2_set_params_t *p)
{
    const unsigned pdu = (p->flags & BC2_FLAG_PDU_MASK) >> BC2_FLAG_PDU_SHIFT;
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.legacy_pdu = 1;                                   /* 31 byte legacy PDU, no AUX channels */
    params.connectable = (pdu == BC2_PDU_ADV_IND);
    params.scannable = (pdu == BC2_PDU_ADV_IND || pdu == BC2_PDU_ADV_SCAN_IND);
    params.itvl_min = p->interval;                           /* 0.625 ms units, min == max: no jitter band */
    params.itvl_max = p->interval;
    params.channel_map = 0x07;                               /* channels 37, 38 and 39 */
    params.own_addr_type = BLE_OWN_ADDR_RANDOM;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_1M;                /* unused for legacy, must still be valid */
    params.tx_power = p->tx_power;                           /* 127 = no preference = controller maximum */
    return params;
}

/** Remove an instance after a failed start (best effort). */
static void abort_start(uint8_t id)
{
    portENTER_CRITICAL(&s_mux);
    s_inst[id].active = false;
    portEXIT_CRITICAL(&s_mux);
    ble_gap_ext_adv_remove(id);
}

static bc2_status_t op_start(void *ctx, uint8_t id, const bc2_set_params_t *p, int8_t *applied_tx,
                             uint16_t *rc_out)
{
    (void)ctx;
    if (!s_synced) {
        *rc_out = RC_NOT_READY;
        return BC2_ERR_BLE_ERROR;
    }
    uint8_t ad[BC2_LEGACY_ADV_MAX];
    const size_t ad_len = bc2_build_adv_data(p->flags, p->mfr_id, p->data, p->data_len, ad);
    if (ad_len == 0) {
        return BC2_ERR_PAYLOAD_TOO_LARGE;
    }

    struct ble_gap_ext_adv_params params = make_params(p);
    ble_addr_t addr;
    make_address(id, &addr);
    int rc = ble_gap_ext_adv_configure(id, &params, applied_tx, gap_event, (void *)(uintptr_t)id);
    if (rc == 0) {
        rc = ble_gap_ext_adv_set_addr(id, &addr);
    }
    if (rc == 0) {
        rc = ble_gap_ext_adv_set_data(id, make_mbuf(ad, ad_len));
    }
    if (rc == 0 && params.scannable) {
        rc = set_empty_scan_response(id);
    }
    if (rc == 0) {
        portENTER_CRITICAL(&s_mux);                  /* active before start: ADV_COMPLETE may come at once */
        s_inst[id].active = true;
        s_inst[id].needs_restart = false;
        s_inst[id].flags = p->flags;
        s_fail[id].pending = false;
        portEXIT_CRITICAL(&s_mux);
        rc = ble_gap_ext_adv_start(id, 0, 0);        /* duration 0, no event limit: until stopped */
    }
    if (rc != 0) {
        abort_start(id);
        *rc_out = rc16(rc);
        return rc == BLE_HS_ENOMEM ? BC2_ERR_NO_RESOURCES : BC2_ERR_BLE_ERROR;
    }
    return BC2_OK;
}

static bc2_status_t op_update(void *ctx, uint8_t id, uint16_t mfr_id, const uint8_t *data, uint8_t len,
                              uint16_t *rc_out)
{
    (void)ctx;
    uint8_t flags;
    portENTER_CRITICAL(&s_mux);
    flags = s_inst[id].flags;
    portEXIT_CRITICAL(&s_mux);

    uint8_t ad[BC2_LEGACY_ADV_MAX];
    const size_t ad_len = bc2_build_adv_data(flags, mfr_id, data, len, ad);
    if (ad_len == 0) {
        return BC2_ERR_PAYLOAD_TOO_LARGE;
    }
    /* Fast path: change the data of the running instance (legal for legacy PDUs). */
    int rc = ble_gap_ext_adv_set_data(id, make_mbuf(ad, ad_len));
    bool fallback_used = false;
    if (rc != 0) {
        fallback_used = true;
        /* Fallback: some controllers refuse changes while enabled -> stop, set data, start. */
        s_fallback_updates++;
        rc = ble_gap_ext_adv_stop(id);
        if (rc == 0 || rc == BLE_HS_EALREADY) {
            rc = ble_gap_ext_adv_set_data(id, make_mbuf(ad, ad_len));
        }
        if (rc == 0) {
            rc = ble_gap_ext_adv_start(id, 0, 0);
        }
    }
    if (rc != 0) {
        *rc_out = rc16(rc);
        if (fallback_used) {
            /* The fallback may have stopped the instance already: the set is silent, so make the
             * core drop the slot (EVT_SET_STOPPED) to keep PONG/active_mask truthful. */
            report_failure(id, BC2_STOP_BLE_ERROR);
        }
        return BC2_ERR_BLE_ERROR;
    }
    return BC2_OK;
}

static bool op_stop(void *ctx, uint8_t id)
{
    (void)ctx;
    portENTER_CRITICAL(&s_mux);
    s_inst[id].active = false;                       /* first: no restart from the host task any more */
    s_inst[id].needs_restart = false;
    portEXIT_CRITICAL(&s_mux);
    int rc = ble_gap_ext_adv_stop(id);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        return false;                                /* may still be advertising: core retries / resets */
    }
    rc = ble_gap_ext_adv_remove(id);                 /* free the instance for new parameters */
    return rc == 0 || rc == BLE_HS_EALREADY || rc == BLE_HS_ENOENT;
}

static bool op_take_failure(void *ctx, uint8_t *id, uint8_t *reason)
{
    (void)ctx;
    bool found = false;
    portENTER_CRITICAL(&s_mux);
    for (uint8_t i = 0; i < MAX_SETS && !found; i++) {
        if (s_fail[i].pending) {
            s_fail[i].pending = false;
            *id = i;
            *reason = s_fail[i].reason;
            found = true;
        }
    }
    portEXIT_CRITICAL(&s_mux);
    return found;
}

/* ------------------------------------------------------------------------- */
/* Initialisation                                                             */
/* ------------------------------------------------------------------------- */

/** Called by NimBLE when the host and controller are in sync. */
static void on_sync(void)
{
    s_synced = true;
}

/** FreeRTOS task running the NimBLE host event loop. */
static void host_task(void *param)
{
    (void)param;
    nimble_port_run();                               /* returns only when nimble_port_stop() is called */
    nimble_port_freertos_deinit();
}

esp_err_t bc2_ble_init(void)
{
    esp_read_mac(s_mac, ESP_MAC_BT);
    const esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        return err;
    }
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

const bc2_ble_ops_t *bc2_ble_ops(void)
{
    static const bc2_ble_ops_t ops = {
        .ctx = NULL,
        .ready = op_ready,
        .max_sets = op_max_sets,
        .device_id = op_device_id,
        .start = op_start,
        .update = op_update,
        .stop = op_stop,
        .take_failure = op_take_failure,
    };
    return &ops;
}
