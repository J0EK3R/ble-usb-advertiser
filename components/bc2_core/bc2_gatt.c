/**
 * @file bc2_gatt.c
 * @brief GATT central / scanner logic, see bc2_gatt.h and docs/protocol.md sections 13-17.
 *
 * Everything here runs under the core mutex, single threaded. Nothing blocks: requests that need
 * the radio are handed to the adapter and completed later by adapter events (bc2_gatt_pump()).
 * Time driven work (deadlines, the connect scheduler, failsafe/terminate sequencing) lives in
 * bc2_gatt_tick(), which the core calls every <= 20 ms.
 */
#include <string.h>

#include "bc2_gatt.h"

/* ------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* ------------------------------------------------------------------------- */

/** True while @p now_ms is before @p t_ms (wrap-safe for spans below 2^31 ms). */
static bool before(uint32_t now_ms, uint32_t t_ms)
{
    return (int32_t)(t_ms - now_ms) > 0;
}

/** True once @p t_ms has been reached. */
static bool reached(uint32_t now_ms, uint32_t t_ms)
{
    return !before(now_ms, t_ms);
}

/** Send a frame through the core; returns false if a droppable frame was refused. */
static bool send(bc2_gatt_t *g, const bc2_frame_t *fr, bool droppable)
{
    return g->out.send(g->out.ctx, fr, droppable);
}

/** Send ACK(orig_type, status, extra). ACKs are never droppable. */
static void send_ack(bc2_gatt_t *g, uint8_t seq, uint8_t type, uint8_t status, const uint8_t *extra,
                     uint8_t extra_len)
{
    bc2_frame_t fr;
    if (bc2_encode_ack(&fr, seq, type, status, extra, extra_len)) {
        send(g, &fr, false);
    }
}

/** ACK for an adapter error: ATT_ERROR carries the ATT code, BLE_ERROR the NimBLE rc. */
static void send_error_ack(bc2_gatt_t *g, uint8_t seq, uint8_t type, uint8_t status, uint16_t code)
{
    const uint8_t extra[2] = {(uint8_t)(code & 0xFFu), (uint8_t)(code >> 8)};
    uint8_t extra_len = 0;
    if (status == BC2_ERR_ATT_ERROR) {
        extra_len = 1;
    } else if (status == BC2_ERR_BLE_ERROR) {
        extra_len = 2;
    }
    send_ack(g, seq, type, status, extra, extra_len);
}

/** Reset a slot to FREE. */
static void slot_free(bc2_gconn_t *c)
{
    memset(c, 0, sizeof(*c));
    c->state = BC2_GC_FREE;
}

/** Send EVT_DISCONNECTED and free the slot. */
static void report_and_free(bc2_gatt_t *g, uint8_t id, uint8_t phase, uint8_t reason)
{
    bc2_frame_t fr;
    bc2_encode_evt_disconnected(&fr, id, phase, reason);
    send(g, &fr, false);
    slot_free(&g->conn[id]);
}

/** Number of slots in use. */
static uint8_t used_slots(const bc2_gatt_t *g)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < BC2_GATT_MAX_CONNS_LIMIT; i++) {
        if (g->conn[i].state != BC2_GC_FREE) {
            n++;
        }
    }
    return n;
}

/** Saturating increment of a u8 counter. */
static uint8_t sat_inc(uint8_t v)
{
    return v == 0xFFu ? v : (uint8_t)(v + 1u);
}

/* ------------------------------------------------------------------------- */
/* Initialisation and capabilities                                            */
/* ------------------------------------------------------------------------- */

void bc2_gatt_init(bc2_gatt_t *g, const bc2_gatt_ops_t *ops, const bc2_gatt_out_t *out)
{
    memset(g, 0, sizeof(*g));
    g->out = *out;
    g->present = ops != NULL;
    if (ops != NULL) {
        g->ops = *ops;
    }
    g->attempt_conn = -1;
}

bool bc2_gatt_present(const bc2_gatt_t *g)
{
    return g->present;
}

uint8_t bc2_gatt_flags(const bc2_gatt_t *g)
{
    return g->present ? (uint8_t)(BC2_GATT_FLAG_SCAN | BC2_GATT_FLAG_CENTRAL | BC2_GATT_FLAG_FAILSAFE) : 0u;
}

uint8_t bc2_gatt_max_conns(const bc2_gatt_t *g)
{
    if (!g->present) {
        return 0;
    }
    const uint8_t n = g->ops.max_conns(g->ops.ctx);
    return n > BC2_GATT_MAX_CONNS_LIMIT ? (uint8_t)BC2_GATT_MAX_CONNS_LIMIT : n;
}

bool bc2_gatt_active(const bc2_gatt_t *g)
{
    return g->present && (g->scanning || used_slots(g) > 0);
}

/* ------------------------------------------------------------------------- */
/* Link termination: failsafe writes, then the terminate request              */
/* ------------------------------------------------------------------------- */

/**
 * Advance the termination sequence of a slot: execute the failsafe writes in slot order (a write
 * refused with BUSY is retried until term_limit_ms), wait BC2_GATT_FAILSAFE_FLUSH_MS after the last
 * write so the data is on air first, then request the termination and repeat the request until the
 * link is gone. Called when the termination starts and from every tick.
 */
static void step_terminate(bc2_gatt_t *g, uint8_t id, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[id];
    if (c->term_sent) {
        if (reached(now, c->term_retry_ms)) {                /* the first request may have been lost */
            g->ops.terminate(g->ops.ctx, id);
            c->term_retry_ms = now + BC2_GATT_TERMINATE_RETRY_MS;
        }
        return;
    }
    while (c->run_failsafe && c->fs_next < BC2_GATT_FAILSAFE_SLOTS) {
        const bc2_failsafe_t *fs = &c->failsafe[c->fs_next];
        if (fs->len == 0) {
            c->fs_next++;                                     /* empty slot */
            continue;
        }
        uint16_t rc = 0;
        const bc2_status_t st = g->ops.write_cmd(g->ops.ctx, id, fs->handle, fs->data, fs->len, &rc);
        if (st == BC2_ERR_BUSY && before(now, c->term_limit_ms)) {
            return;                                           /* controller full: retry in the next tick */
        }
        c->fs_next++;                                         /* written, or given up (error / out of time) */
        if (st == BC2_OK) {
            c->term_due_ms = now + BC2_GATT_FAILSAFE_FLUSH_MS;
        }
    }
    if (reached(now, c->term_due_ms)) {
        g->ops.terminate(g->ops.ctx, id);
        c->term_sent = true;
        c->term_retry_ms = now + BC2_GATT_TERMINATE_RETRY_MS;
    }
}

/**
 * Start terminating a connected link.
 * @param reason      value for EVT_DISCONNECTED (0 = report the stack's own reason)
 * @param run_failsafe execute the stored failsafe writes first
 */
static void begin_terminate(bc2_gatt_t *g, uint8_t id, uint8_t reason, bool run_failsafe, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[id];
    c->state = BC2_GC_TERMINATING;
    c->term_reason = reason;
    c->run_failsafe = run_failsafe;
    c->fs_next = 0;
    c->term_sent = false;
    c->term_due_ms = now;                                     /* no failsafe write: terminate at once */
    c->term_limit_ms = now + BC2_GATT_FAILSAFE_MAX_WAIT_MS;
    step_terminate(g, id, now);
}

/** Answer the pending procedure of a slot with a failure status (link lost / timeout) and clear it. */
static void fail_pending(bc2_gatt_t *g, uint8_t id, uint8_t status)
{
    bc2_gconn_t *c = &g->conn[id];
    if (!c->pend_active) {
        return;
    }
    c->pend_active = false;
    if (c->pend_type == BC2_MSG_DISCOVER) {                   /* discovery was already ACKed: finish with the event */
        bc2_frame_t fr;
        bc2_encode_evt_discover_done(&fr, id, status, c->n_services, c->n_chars);
        send(g, &fr, false);
    } else {
        send_ack(g, c->pend_seq, c->pend_type, status, NULL, 0);
    }
}

/* ------------------------------------------------------------------------- */
/* Connect scheduler (the controller has a single initiator)                  */
/* ------------------------------------------------------------------------- */

/** Ask the adapter to cancel the running attempt of slot @p id. */
static void cancel_attempt(bc2_gatt_t *g, uint8_t id, uint8_t reason, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[id];
    c->state = BC2_GC_CANCEL;
    c->cancel_reason = reason;
    g->cancel_sent_ms = now;
    g->ops.connect_cancel(g->ops.ctx);
}

/**
 * The attempt of slot @p id ended without a link (failure, cancel, slice expiry).
 * Cancelled for real: report and free. Otherwise retry until the overall deadline.
 */
static void attempt_ended(bc2_gatt_t *g, uint8_t id, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[id];
    if (g->attempt_conn == (int8_t)id) {
        g->attempt_conn = -1;
    }
    if (c->state == BC2_GC_CANCEL && c->cancel_reason != 0) {
        report_and_free(g, id, BC2_PHASE_CONNECTING, c->cancel_reason);
    } else if (c->has_deadline && reached(now, c->deadline_ms)) {
        report_and_free(g, id, BC2_PHASE_CONNECTING, BC2_REASON_CONNECT_TIMEOUT);
    } else {
        const bool slice_expired = c->state == BC2_GC_CANCEL;  /* yielded to others: no pause needed */
        c->state = BC2_GC_WAIT;
        c->cancel_reason = 0;
        c->retry_at_ms = slice_expired ? now : now + BC2_GATT_RETRY_DELAY_MS;
    }
}

/** True if a slot other than @p except waits for the initiator and is due. */
static bool others_waiting(const bc2_gatt_t *g, uint8_t except, uint32_t now)
{
    for (uint8_t i = 0; i < BC2_GATT_MAX_CONNS_LIMIT; i++) {
        if (i != except && g->conn[i].state == BC2_GC_WAIT && reached(now, g->conn[i].retry_at_ms)) {
            return true;
        }
    }
    return false;
}

/** Start the next connect attempt if the initiator is free (round robin over the waiting slots). */
static void schedule_connect(bc2_gatt_t *g, uint32_t now)
{
    if (g->attempt_conn >= 0) {
        return;
    }
    for (uint8_t k = 0; k < BC2_GATT_MAX_CONNS_LIMIT; k++) {
        const uint8_t id = (uint8_t)((g->rr_next + k) % BC2_GATT_MAX_CONNS_LIMIT);
        bc2_gconn_t *c = &g->conn[id];
        if (c->state != BC2_GC_WAIT || !reached(now, c->retry_at_ms)) {
            continue;
        }
        /* Background connects run until cancelled; others for the rest of their deadline. */
        uint32_t duration = 0;
        if (c->has_deadline) {
            duration = reached(now, c->deadline_ms) ? 1u : (uint32_t)(c->deadline_ms - now);
        }
        uint16_t rc = 0;
        const bc2_status_t st = g->ops.connect(g->ops.ctx, id, c->addr, c->addr_type, c->itvl_min,
                                               c->itvl_max, duration, &rc);
        if (st == BC2_OK) {
            c->state = BC2_GC_ATTEMPT;
            c->attempt_start_ms = now;
            g->attempt_conn = (int8_t)id;
            g->rr_next = (uint8_t)((id + 1u) % BC2_GATT_MAX_CONNS_LIMIT);
        } else if (st == BC2_ERR_BUSY) {
            c->retry_at_ms = now + 100u;                      /* initiator busy: try again shortly */
        } else {
            report_and_free(g, id, BC2_PHASE_CONNECTING, BC2_REASON_INTERNAL);
        }
        return;                                               /* one attempt (or one failure) per call */
    }
}

/* ------------------------------------------------------------------------- */
/* Request handlers                                                           */
/* ------------------------------------------------------------------------- */

static void handle_scan_start(bc2_gatt_t *g, const bc2_frame_t *fr)
{
    bc2_req_scan_start_t req;
    bc2_status_t st = bc2_decode_scan_start(fr, &req);
    if (st == BC2_OK && g->scanning) {
        st = BC2_ERR_ALREADY_RUNNING;
    }
    uint16_t rc = 0;
    if (st == BC2_OK) {
        st = g->ops.scan_start(g->ops.ctx, &req, &rc);
        g->scanning = st == BC2_OK;
    }
    send_error_ack(g, fr->seq, fr->type, st, rc);
}

static void handle_scan_stop(bc2_gatt_t *g, const bc2_frame_t *fr)
{
    const bc2_status_t st = bc2_decode_empty(fr);
    if (st == BC2_OK && g->scanning) {
        g->ops.scan_stop(g->ops.ctx);
        g->scanning = false;
    }
    send_ack(g, fr->seq, fr->type, st, NULL, 0);
}

static void handle_connect(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    bc2_req_connect_t req;
    bc2_status_t st = bc2_decode_connect(fr, &req);
    if (st == BC2_OK && g->conn[req.conn_id].state != BC2_GC_FREE) {
        st = BC2_ERR_ALREADY_RUNNING;
    }
    for (uint8_t i = 0; st == BC2_OK && i < BC2_GATT_MAX_CONNS_LIMIT; i++) {   /* same device twice? */
        if (g->conn[i].state != BC2_GC_FREE && memcmp(g->conn[i].addr, req.addr, 6) == 0) {
            st = BC2_ERR_ALREADY_RUNNING;
        }
    }
    if (st == BC2_OK && used_slots(g) >= bc2_gatt_max_conns(g)) {
        st = BC2_ERR_NO_RESOURCES;
    }
    if (st != BC2_OK) {
        send_ack(g, fr->seq, fr->type, st, NULL, 0);
        return;
    }
    bc2_gconn_t *c = &g->conn[req.conn_id];
    slot_free(c);
    c->state = BC2_GC_WAIT;
    memcpy(c->addr, req.addr, 6);
    c->addr_type = req.addr_type;
    c->background = (req.flags & BC2_CONNECT_FLAG_BACKGROUND) != 0;
    c->has_deadline = !c->background;
    c->deadline_ms = now + req.timeout_ms;
    c->itvl_min = req.itvl_min;
    c->itvl_max = req.itvl_max;
    c->retry_at_ms = now;
    send_ack(g, fr->seq, fr->type, BC2_OK, NULL, 0);          /* ACK first: events follow later */
    schedule_connect(g, now);
}

static void handle_disconnect(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    uint8_t id = 0;
    const bc2_status_t st = bc2_decode_conn_id(fr, &id);
    send_ack(g, fr->seq, fr->type, st, NULL, 0);              /* the ACK comes before the events it causes */
    if (st != BC2_OK) {
        return;
    }
    bc2_gconn_t *c = &g->conn[id];
    switch (c->state) {
    case BC2_GC_WAIT:
        report_and_free(g, id, BC2_PHASE_CONNECTING, BC2_REASON_CANCELLED);
        break;
    case BC2_GC_ATTEMPT:
        cancel_attempt(g, id, BC2_REASON_CANCELLED, now);
        break;
    case BC2_GC_CANCEL:
        c->cancel_reason = BC2_REASON_CANCELLED;              /* a pure requeue becomes a real cancel */
        break;
    case BC2_GC_CONNECTED:
        begin_terminate(g, id, 0, false, now);                /* host cleans up itself: no failsafe */
        break;
    default:                                                  /* FREE or already terminating: OK, nothing to do */
        break;
    }
}

/** Look up the link of a request; sets @p st to NOT_CONNECTED unless it is connected. */
static bc2_gconn_t *connected_slot(bc2_gatt_t *g, uint8_t id, bc2_status_t *st)
{
    if (g->conn[id].state != BC2_GC_CONNECTED) {
        *st = BC2_ERR_NOT_CONNECTED;
        return NULL;
    }
    return &g->conn[id];
}

/** Mark a procedure as running on @p c (the deadline restarts with every discovery event). */
static void start_procedure(bc2_gconn_t *c, uint8_t type, uint8_t seq, uint16_t handle, uint32_t now)
{
    c->pend_active = true;
    c->pend_type = type;
    c->pend_seq = seq;
    c->pend_handle = handle;
    c->pend_deadline_ms = now + BC2_GATT_PROC_TIMEOUT_MS;
}

static void handle_discover(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    uint8_t id = 0;
    bc2_status_t st = bc2_decode_conn_id(fr, &id);
    bc2_gconn_t *c = st == BC2_OK ? connected_slot(g, id, &st) : NULL;
    if (st == BC2_OK && c->pend_active) {
        st = BC2_ERR_BUSY;
    }
    uint16_t rc = 0;
    if (st == BC2_OK) {
        st = g->ops.discover(g->ops.ctx, id, &rc);
    }
    if (st == BC2_OK) {
        start_procedure(c, BC2_MSG_DISCOVER, fr->seq, 0, now);
        c->n_services = 0;
        c->n_chars = 0;
    }
    send_error_ack(g, fr->seq, fr->type, st, rc);
}

static void handle_read(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    bc2_req_read_t req;
    bc2_status_t st = bc2_decode_read(fr, &req);
    bc2_gconn_t *c = st == BC2_OK ? connected_slot(g, req.conn_id, &st) : NULL;
    if (st == BC2_OK && c->pend_active) {
        st = BC2_ERR_BUSY;
    }
    uint16_t rc = 0;
    if (st == BC2_OK) {
        st = g->ops.read(g->ops.ctx, req.conn_id, req.handle, &rc);
    }
    if (st == BC2_OK) {
        start_procedure(c, BC2_MSG_READ, fr->seq, req.handle, now);   /* READ_RSP answers later */
        return;
    }
    send_error_ack(g, fr->seq, fr->type, st, rc);
}

/** WRITE (with response) and WRITE_CMD (without) share their checks. */
static void handle_write(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    const bool with_response = fr->type == BC2_MSG_WRITE;
    bc2_req_write_t req;
    bc2_status_t st = bc2_decode_write(fr, &req);
    bc2_gconn_t *c = st == BC2_OK ? connected_slot(g, req.conn_id, &st) : NULL;
    if (st == BC2_OK && req.data_len + 3u > c->mtu) {         /* ATT payload is mtu - 3 bytes */
        st = BC2_ERR_PAYLOAD_TOO_LARGE;
    }
    if (st == BC2_OK && with_response && c->pend_active) {    /* WRITE_CMD is not a procedure */
        st = BC2_ERR_BUSY;
    }
    uint16_t rc = 0;
    if (st == BC2_OK) {
        st = with_response ? g->ops.write(g->ops.ctx, req.conn_id, req.handle, req.data, req.data_len, &rc)
                           : g->ops.write_cmd(g->ops.ctx, req.conn_id, req.handle, req.data, req.data_len, &rc);
    }
    if (st == BC2_OK && with_response) {
        start_procedure(c, BC2_MSG_WRITE, fr->seq, req.handle, now);   /* ACK after the ATT response */
        return;
    }
    send_error_ack(g, fr->seq, fr->type, st, rc);
}

static void handle_subscribe(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    bc2_req_subscribe_t req;
    bc2_status_t st = bc2_decode_subscribe(fr, &req);
    bc2_gconn_t *c = st == BC2_OK ? connected_slot(g, req.conn_id, &st) : NULL;
    if (st == BC2_OK && c->pend_active) {
        st = BC2_ERR_BUSY;
    }
    uint16_t rc = 0;
    if (st == BC2_OK) {
        st = g->ops.subscribe(g->ops.ctx, req.conn_id, req.cccd_handle, req.mode, &rc);
    }
    if (st == BC2_OK) {
        start_procedure(c, BC2_MSG_SUBSCRIBE, fr->seq, req.cccd_handle, now);
        return;
    }
    send_error_ack(g, fr->seq, fr->type, st, rc);
}

/** Fetch the adapter's counters and heap figures; returns true if a state event was lost. */
static bool poll_adapter_stats(bc2_gatt_t *g)
{
    bc2_gatt_adapter_stats_t s;
    memset(&s, 0, sizeof(s));
    g->ops.stats(g->ops.ctx, &s);
    g->dropped_scan = (uint16_t)(g->dropped_scan + s.dropped_scan);
    g->dropped_notify = (uint16_t)(g->dropped_notify + s.dropped_notify);
    g->free_heap = s.free_heap;
    g->min_free_heap = s.min_free_heap;
    return s.state_lost;
}

static void handle_get_status(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now)
{
    const bc2_status_t st = bc2_decode_empty(fr);
    if (st != BC2_OK) {
        send_ack(g, fr->seq, fr->type, st, NULL, 0);
        return;
    }
    if (poll_adapter_stats(g)) {
        bc2_gatt_safety_stop(g, now, BC2_REASON_INTERNAL);
    }
    bc2_gatt_status_t out = {.scanning = g->scanning ? 1u : 0u, .dropped_scan = g->dropped_scan,
                             .dropped_notify = g->dropped_notify, .free_heap = g->free_heap,
                             .min_free_heap = g->min_free_heap};
    for (uint8_t i = 0; i < BC2_GATT_MAX_CONNS_LIMIT; i++) {
        const uint8_t s = g->conn[i].state;
        if (s == BC2_GC_WAIT || s == BC2_GC_ATTEMPT || s == BC2_GC_CANCEL) {
            out.connecting_mask = (uint16_t)(out.connecting_mask | (1u << i));
        } else if (s == BC2_GC_CONNECTED) {
            out.connected_mask = (uint16_t)(out.connected_mask | (1u << i));
        }
    }
    bc2_frame_t rsp;
    bc2_encode_gatt_status_rsp(&rsp, fr->seq, &out);
    send(g, &rsp, false);
}

static void handle_set_failsafe(bc2_gatt_t *g, const bc2_frame_t *fr)
{
    bc2_req_set_failsafe_t req;
    bc2_status_t st = bc2_decode_set_failsafe(fr, &req);
    bc2_gconn_t *c = st == BC2_OK ? connected_slot(g, req.conn_id, &st) : NULL;
    if (st == BC2_OK) {
        bc2_failsafe_t *fs = &c->failsafe[req.slot];
        fs->len = req.data_len;                               /* 0 clears the slot */
        fs->handle = req.data_len > 0 ? req.handle : 0;
        memcpy(fs->data, req.data, req.data_len);
    }
    send_ack(g, fr->seq, fr->type, st, NULL, 0);
}

void bc2_gatt_handle_request(bc2_gatt_t *g, const bc2_frame_t *fr, uint32_t now_ms)
{
    if (!g->present) {
        send_ack(g, fr->seq, fr->type, BC2_ERR_NOT_SUPPORTED, NULL, 0);
        return;
    }
    switch (fr->type) {
    case BC2_MSG_SCAN_START:
        handle_scan_start(g, fr);
        break;
    case BC2_MSG_SCAN_STOP:
        handle_scan_stop(g, fr);
        break;
    case BC2_MSG_CONNECT:
        handle_connect(g, fr, now_ms);
        break;
    case BC2_MSG_DISCONNECT:
        handle_disconnect(g, fr, now_ms);
        break;
    case BC2_MSG_DISCOVER:
        handle_discover(g, fr, now_ms);
        break;
    case BC2_MSG_READ:
        handle_read(g, fr, now_ms);
        break;
    case BC2_MSG_WRITE:
    case BC2_MSG_WRITE_CMD:
        handle_write(g, fr, now_ms);
        break;
    case BC2_MSG_SUBSCRIBE:
        handle_subscribe(g, fr, now_ms);
        break;
    case BC2_MSG_GET_GATT_STATUS:
        handle_get_status(g, fr, now_ms);
        break;
    default:   /* BC2_MSG_SET_FAILSAFE, the last request of the range */
        handle_set_failsafe(g, fr);
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Adapter events                                                             */
/* ------------------------------------------------------------------------- */

static void on_link_up(bc2_gatt_t *g, const bc2_gatt_event_t *ev, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[ev->conn_id];
    if (c->state != BC2_GC_ATTEMPT && c->state != BC2_GC_CANCEL) {
        g->ops.terminate(g->ops.ctx, ev->conn_id);            /* a link nobody asked for: never keep it */
        return;
    }
    if (g->attempt_conn == (int8_t)ev->conn_id) {
        g->attempt_conn = -1;
    }
    if (c->state == BC2_GC_CANCEL && c->cancel_reason != 0) {
        begin_terminate(g, ev->conn_id, c->cancel_reason, false, now);   /* cancel lost the race: tear it down */
        return;
    }
    c->state = BC2_GC_CONNECTED;
    c->announced = true;
    c->mtu = ev->u16a;
    bc2_frame_t fr;
    bc2_encode_evt_connected(&fr, ev->conn_id, ev->u16a, ev->u16b, ev->addr);
    send(g, &fr, false);
}

static void on_link_down(bc2_gatt_t *g, const bc2_gatt_event_t *ev, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[ev->conn_id];
    if (c->state == BC2_GC_ATTEMPT || c->state == BC2_GC_CANCEL) {
        attempt_ended(g, ev->conn_id, now);                   /* the link dropped before it was announced */
    } else if (c->state == BC2_GC_CONNECTED || c->state == BC2_GC_TERMINATING) {
        fail_pending(g, ev->conn_id, BC2_ERR_DISCONNECTED);
        report_and_free(g, ev->conn_id, c->announced ? BC2_PHASE_CONNECTED : BC2_PHASE_CONNECTING,
                        c->term_reason != 0 ? c->term_reason : (uint8_t)ev->code);
    }                                                         /* FREE / WAIT: stale event, ignore */
}

static void on_connect_failed(bc2_gatt_t *g, const bc2_gatt_event_t *ev, uint32_t now)
{
    const bc2_gconn_t *c = &g->conn[ev->conn_id];
    if (c->state == BC2_GC_ATTEMPT || c->state == BC2_GC_CANCEL) {
        attempt_ended(g, ev->conn_id, now);
    }
}

static void on_op_done(bc2_gatt_t *g, const bc2_gatt_event_t *ev)
{
    bc2_gconn_t *c = &g->conn[ev->conn_id];
    if (c->state != BC2_GC_CONNECTED || !c->pend_active || c->pend_type == BC2_MSG_DISCOVER) {
        return;                                               /* late completion after a timeout */
    }
    c->pend_active = false;
    if (ev->status == BC2_OK && c->pend_type == BC2_MSG_READ) {
        bc2_frame_t fr;
        if (bc2_encode_read_rsp(&fr, c->pend_seq, ev->conn_id, c->pend_handle,
                                (uint8_t)(ev->flags & BC2_READ_TRUNCATED), ev->data, ev->len)) {
            send(g, &fr, false);
        }
    } else {
        send_error_ack(g, c->pend_seq, c->pend_type, ev->status, ev->code);
    }
}

/** Forward a discovery event and restart the procedure deadline. */
static void on_discovery_item(bc2_gatt_t *g, const bc2_gatt_event_t *ev, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[ev->conn_id];
    if (c->state != BC2_GC_CONNECTED || !c->pend_active || c->pend_type != BC2_MSG_DISCOVER) {
        return;
    }
    bc2_frame_t fr;
    bool ok;
    if (ev->kind == BC2_GEV_SERVICE) {
        ok = bc2_encode_evt_gatt_service(&fr, ev->conn_id, ev->u16a, ev->u16b, ev->data, ev->len);
        c->n_services = sat_inc(c->n_services);
    } else {
        ok = bc2_encode_evt_gatt_char(&fr, ev->conn_id, ev->u16a, ev->u16b, (uint8_t)ev->code, ev->data, ev->len);
        c->n_chars = sat_inc(c->n_chars);
    }
    if (ok) {
        send(g, &fr, false);
    }
    c->pend_deadline_ms = now + BC2_GATT_PROC_TIMEOUT_MS;
}

static void on_discover_done(bc2_gatt_t *g, const bc2_gatt_event_t *ev)
{
    bc2_gconn_t *c = &g->conn[ev->conn_id];
    if (c->state != BC2_GC_CONNECTED || !c->pend_active || c->pend_type != BC2_MSG_DISCOVER) {
        return;
    }
    c->pend_active = false;
    bc2_frame_t fr;
    bc2_encode_evt_discover_done(&fr, ev->conn_id, ev->status, c->n_services, c->n_chars);
    send(g, &fr, false);
}

/** Dispatch one adapter event. */
static void on_event(bc2_gatt_t *g, const bc2_gatt_event_t *ev, uint32_t now)
{
    if (ev->conn_id >= BC2_GATT_MAX_CONNS_LIMIT && ev->kind != BC2_GEV_SCAN_REPORT &&
        ev->kind != BC2_GEV_SCAN_STOPPED) {
        return;                                               /* corrupt event */
    }
    bc2_frame_t fr;
    switch (ev->kind) {
    case BC2_GEV_LINK_UP:
        on_link_up(g, ev, now);
        break;
    case BC2_GEV_LINK_DOWN:
        on_link_down(g, ev, now);
        break;
    case BC2_GEV_CONNECT_FAILED:
        on_connect_failed(g, ev, now);
        break;
    case BC2_GEV_OP_DONE:
        on_op_done(g, ev);
        break;
    case BC2_GEV_NOTIFY:
        if (g->conn[ev->conn_id].state == BC2_GC_CONNECTED &&
            bc2_encode_evt_notify(&fr, ev->conn_id, ev->u16a, ev->flags, ev->data, ev->len) &&
            !send(g, &fr, true)) {
            g->dropped_notify++;                              /* TX queue pressure: counted, reported later */
        }
        break;
    case BC2_GEV_SCAN_REPORT:
        if (g->scanning &&
            bc2_encode_evt_scan_report(&fr, ev->addr, ev->addr_type, ev->rssi, ev->flags, ev->data, ev->len) &&
            !send(g, &fr, true)) {
            g->dropped_scan++;
        }
        break;
    case BC2_GEV_SERVICE:
    case BC2_GEV_CHAR:
        on_discovery_item(g, ev, now);
        break;
    case BC2_GEV_DISCOVER_DONE:
        on_discover_done(g, ev);
        break;
    default:   /* BC2_GEV_SCAN_STOPPED */
        if (g->scanning) {
            g->scanning = false;
            bc2_encode_evt_scan_stopped(&fr, BC2_SCAN_STOP_ERROR, ev->code);
            send(g, &fr, false);
        }
        break;
    }
}

void bc2_gatt_pump(bc2_gatt_t *g, uint32_t now_ms)
{
    if (!g->present) {
        return;
    }
    bc2_gatt_event_t ev;
    for (unsigned n = 0; n < BC2_GATT_MAX_EVENTS_PER_PUMP && g->ops.poll_event(g->ops.ctx, &ev); n++) {
        on_event(g, &ev, now_ms);
    }
}

/* ------------------------------------------------------------------------- */
/* Safety stop                                                                */
/* ------------------------------------------------------------------------- */

void bc2_gatt_safety_stop(bc2_gatt_t *g, uint32_t now_ms, uint8_t reason)
{
    if (!g->present) {
        return;
    }
    if (g->scanning) {
        g->ops.scan_stop(g->ops.ctx);
        g->scanning = false;
        bc2_frame_t fr;
        bc2_encode_evt_scan_stopped(&fr, BC2_SCAN_STOP_SAFETY, 0);
        send(g, &fr, false);
    }
    for (uint8_t id = 0; id < BC2_GATT_MAX_CONNS_LIMIT; id++) {
        bc2_gconn_t *c = &g->conn[id];
        switch (c->state) {
        case BC2_GC_WAIT:
            report_and_free(g, id, BC2_PHASE_CONNECTING, reason);
            break;
        case BC2_GC_ATTEMPT:
            cancel_attempt(g, id, reason, now_ms);
            /* The adapter may already own a link (MTU wait of up to 2 s) that connect_cancel cannot stop:
             * terminate() covers both the pending connect and an existing link. */
            g->ops.terminate(g->ops.ctx, id);
            c->safety_kill = true;
            break;
        case BC2_GC_CANCEL:
            c->cancel_reason = reason;
            g->ops.terminate(g->ops.ctx, id);                 /* same reason: the link may be in the MTU wait */
            c->safety_kill = true;
            break;
        case BC2_GC_CONNECTED:
            begin_terminate(g, id, reason, true, now_ms);     /* failsafe writes, then terminate */
            c->safety_kill = true;
            break;
        case BC2_GC_TERMINATING:
            c->term_reason = reason;
            c->safety_kill = true;
            break;
        default:
            break;
        }
    }
    for (uint8_t id = 0; id < BC2_GATT_MAX_CONNS_LIMIT; id++) {   /* anything left to verify? */
        if (g->conn[id].safety_kill) {
            g->verify_stage = 1;
            g->verify_at_ms = now_ms + BC2_GATT_VERIFY_MS;
            break;
        }
    }
}

void bc2_gatt_session_reset(bc2_gatt_t *g, uint32_t now_ms)
{
    bc2_gatt_safety_stop(g, now_ms, BC2_REASON_SAFETY);
    g->dropped_scan = 0;
    g->dropped_notify = 0;
    g->reported_scan = 0;
    g->reported_notify = 0;
}

/** Verify that the links killed by a safety stop are really gone (stage 1: retry, stage 2: reset the chip). */
static void verify_links_gone(bc2_gatt_t *g, uint32_t now)
{
    if (g->verify_stage == 0 || before(now, g->verify_at_ms)) {
        return;
    }
    bool survivors = false;
    for (uint8_t id = 0; id < BC2_GATT_MAX_CONNS_LIMIT; id++) {
        bc2_gconn_t *c = &g->conn[id];
        if (!c->safety_kill || (c->state != BC2_GC_TERMINATING && c->state != BC2_GC_CANCEL)) {
            continue;
        }
        survivors = true;
        if (g->verify_stage == 1) {                           /* first miss: ask again */
            if (c->state == BC2_GC_CANCEL) {
                g->ops.connect_cancel(g->ops.ctx);
            }
            g->ops.terminate(g->ops.ctx, id);                 /* also ends a link in the adapter's MTU wait */
        }
    }
    if (!survivors) {
        g->verify_stage = 0;
    } else if (g->verify_stage == 1) {
        g->verify_stage = 2;
        g->verify_at_ms = now + BC2_GATT_VERIFY_MS;
    } else {
        g->verify_stage = 0;
        g->out.hard_reset(g->out.ctx);                        /* silence is more important than the session */
    }
}

/* ------------------------------------------------------------------------- */
/* Periodic work                                                              */
/* ------------------------------------------------------------------------- */

/** Report new drops of one kind (at most once per BC2_GATT_DROP_REPORT_MS). */
static void report_drops(bc2_gatt_t *g, uint8_t kind, uint16_t total, uint16_t *reported)
{
    const uint16_t delta = (uint16_t)(total - *reported);
    if (delta == 0) {
        return;
    }
    bc2_frame_t fr;
    bc2_encode_evt_dropped(&fr, kind, delta);
    send(g, &fr, false);
    *reported = total;
}

/** Time driven part of one slot. */
static void tick_slot(bc2_gatt_t *g, uint8_t id, uint32_t now)
{
    bc2_gconn_t *c = &g->conn[id];
    switch (c->state) {
    case BC2_GC_WAIT:
        if (c->has_deadline && reached(now, c->deadline_ms)) {
            report_and_free(g, id, BC2_PHASE_CONNECTING, BC2_REASON_CONNECT_TIMEOUT);
        }
        break;
    case BC2_GC_ATTEMPT:
        if (c->has_deadline && reached(now, c->deadline_ms)) {
            cancel_attempt(g, id, BC2_REASON_CONNECT_TIMEOUT, now);
        } else if ((uint32_t)(now - c->attempt_start_ms) >= BC2_GATT_ATTEMPT_SLICE_MS &&
                   others_waiting(g, id, now)) {
            cancel_attempt(g, id, 0, now);                    /* yield: requeued behind the others */
        }
        break;
    case BC2_GC_CANCEL:
        if (g->attempt_conn == (int8_t)id && (uint32_t)(now - g->cancel_sent_ms) >= BC2_GATT_CANCEL_GUARD_MS) {
            g->ops.terminate(g->ops.ctx, id);                 /* no answer: do not let a late link survive */
            attempt_ended(g, id, now);
        }
        break;
    case BC2_GC_CONNECTED:
        if (c->pend_active && reached(now, c->pend_deadline_ms)) {
            fail_pending(g, id, BC2_ERR_TIMEOUT);
            begin_terminate(g, id, BC2_REASON_INTERNAL, true, now);   /* stuck link: failsafe, then drop it */
        }
        break;
    case BC2_GC_TERMINATING:
        step_terminate(g, id, now);
        break;
    default:
        break;
    }
}

void bc2_gatt_tick(bc2_gatt_t *g, uint32_t now_ms)
{
    if (!g->present) {
        return;
    }
    if (poll_adapter_stats(g)) {
        bc2_gatt_safety_stop(g, now_ms, BC2_REASON_INTERNAL);   /* link state unknown: drop everything */
    }
    for (uint8_t id = 0; id < BC2_GATT_MAX_CONNS_LIMIT; id++) {
        tick_slot(g, id, now_ms);
    }
    schedule_connect(g, now_ms);
    verify_links_gone(g, now_ms);
    if (reached(now_ms, g->last_drop_report_ms + BC2_GATT_DROP_REPORT_MS)) {
        report_drops(g, BC2_DROP_SCAN, g->dropped_scan, &g->reported_scan);
        report_drops(g, BC2_DROP_NOTIFY, g->dropped_notify, &g->reported_notify);
        g->last_drop_report_ms = now_ms;
    }
}

bool bc2_gatt_descriptor_range(uint16_t value_handle, bool has_next, uint16_t next_def_handle,
                               uint16_t service_end, uint16_t *first, uint16_t *last)
{
    /* 32 bit arithmetic: value_handle + 1 must not wrap when the value handle is 0xFFFF */
    const uint32_t lo = (uint32_t)value_handle + 1u;
    uint32_t hi = service_end;
    if (has_next) {
        hi = next_def_handle > 0u ? (uint32_t)next_def_handle - 1u : 0u;   /* handle 0 is invalid: empty range */
    }
    if (lo > hi || lo > 0xFFFFu) {
        return false;
    }
    *first = (uint16_t)lo;
    *last = (uint16_t)hi;
    return true;
}
