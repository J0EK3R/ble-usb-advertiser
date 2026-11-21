/**
 * @file fake_ble.c
 * @brief Implementation of the fake BLE adapter / IO used by the core tests.
 */
#include <string.h>

#include "fake_ble.h"

/* ---- fake bc2_ble_ops_t ---- */

static bool f_ready(void *ctx) { return ((fake_env_t *)ctx)->ready; }
static uint8_t f_max_sets(void *ctx) { return ((fake_env_t *)ctx)->max_sets; }

static void f_device_id(void *ctx, uint8_t out[6])
{
    (void)ctx;
    static const uint8_t id[6] = {0x24, 0x0A, 0xBC, 0x11, 0x22, 0x33};
    memcpy(out, id, 6);
}

/** Record a call in the log (silently ignored when the log is full). */
static fake_call_t *record(fake_env_t *env, fake_call_kind_t kind, uint8_t set_id)
{
    if (env->call_count >= FAKE_MAX_CALLS) {
        return NULL;
    }
    fake_call_t *c = &env->calls[env->call_count++];
    memset(c, 0, sizeof(*c));
    c->kind = kind;
    c->set_id = set_id;
    return c;
}

static bc2_status_t f_start(void *ctx, uint8_t set_id, const bc2_set_params_t *p, int8_t *applied_tx,
                            uint16_t *rc)
{
    fake_env_t *env = ctx;
    fake_call_t *c = record(env, CALL_START, set_id);
    if (c != NULL) {
        c->mfr_id = p->mfr_id;
        c->flags = p->flags;
        c->data_len = p->data_len;
        memcpy(c->data, p->data, p->data_len);
    }
    if (env->start_result != BC2_OK) {
        *rc = env->error_rc;
        return env->start_result;
    }
    *applied_tx = 9;   /* pretend the controller clamped 127 to +9 dBm */
    return BC2_OK;
}

static bc2_status_t f_update(void *ctx, uint8_t set_id, uint16_t mfr_id, const uint8_t *data,
                             uint8_t len, uint16_t *rc)
{
    fake_env_t *env = ctx;
    fake_call_t *c = record(env, CALL_UPDATE, set_id);
    if (c != NULL) {
        c->mfr_id = mfr_id;
        c->data_len = len;
        memcpy(c->data, data, len);
    }
    if (env->update_result != BC2_OK) {
        *rc = env->error_rc;
    }
    return env->update_result;
}

static bool f_stop(void *ctx, uint8_t set_id)
{
    fake_env_t *env = ctx;
    record(env, CALL_STOP, set_id);
    if (env->stop_failures > 0) {
        env->stop_failures--;
        return false;
    }
    return true;
}

static bool f_take_failure(void *ctx, uint8_t *set_id, uint8_t *reason)
{
    fake_env_t *env = ctx;
    if (!env->failure_pending) {
        return false;
    }
    env->failure_pending = false;
    *set_id = env->failure_set;
    *reason = env->failure_reason;
    return true;
}

/* ---- fake bc2_io_ops_t ---- */

static void f_write(void *ctx, const uint8_t *data, size_t len)
{
    fake_env_t *env = ctx;
    if (env->out_len + len <= sizeof(env->out)) {
        memcpy(&env->out[env->out_len], data, len);
        env->out_len += len;
    }
}

static uint32_t f_now_us(void *ctx)
{
    fake_env_t *env = ctx;
    const uint32_t v = env->now_us;
    env->now_us += env->us_per_call;
    return v;
}

static void f_hard_reset(void *ctx) { ((fake_env_t *)ctx)->hard_resets++; }

static bool f_write_droppable(void *ctx, const uint8_t *data, size_t len)
{
    fake_env_t *env = ctx;
    if (env->refuse_droppable) {
        return false;
    }
    f_write(ctx, data, len);
    return true;
}

/* ---- fake bc2_gatt_ops_t ---- */

/** Record a GATT call; returns the entry (or a scratch entry when the log is full). */
static fake_gcall_t *grecord(fake_env_t *env, fake_gcall_kind_t kind, uint8_t conn_id)
{
    static fake_gcall_t scratch;
    fake_gcall_t *c = env->gcall_count < FAKE_MAX_GCALLS ? &env->gcalls[env->gcall_count++] : &scratch;
    memset(c, 0, sizeof(*c));
    c->kind = kind;
    c->conn_id = conn_id;
    c->at_ms = env->now_ms;
    return c;
}

static uint8_t g_max_conns(void *ctx) { return ((fake_env_t *)ctx)->max_conns; }

static bc2_status_t g_scan_start(void *ctx, const bc2_req_scan_start_t *p, uint16_t *rc)
{
    fake_env_t *env = ctx;
    fake_gcall_t *c = grecord(env, GCALL_SCAN_START, 0);
    c->mode = p->flags;
    c->itvl_min = p->interval;
    c->itvl_max = p->window;
    c->handle = p->refresh_ms;
    *rc = env->g_rc;
    return env->gresult[GCALL_SCAN_START];
}

static void g_scan_stop(void *ctx) { grecord(ctx, GCALL_SCAN_STOP, 0); }

static bc2_status_t g_connect(void *ctx, uint8_t conn_id, const uint8_t addr[6], uint8_t addr_type,
                              uint16_t itvl_min, uint16_t itvl_max, uint32_t duration_ms, uint16_t *rc)
{
    fake_env_t *env = ctx;
    fake_gcall_t *c = grecord(env, GCALL_CONNECT, conn_id);
    memcpy(c->addr, addr, 6);
    c->mode = addr_type;
    c->itvl_min = itvl_min;
    c->itvl_max = itvl_max;
    c->duration_ms = duration_ms;
    *rc = env->g_rc;
    return env->gresult[GCALL_CONNECT];
}

static void g_connect_cancel(void *ctx) { grecord(ctx, GCALL_CANCEL, 0xFF); }

static bool g_terminate(void *ctx, uint8_t conn_id)
{
    grecord(ctx, GCALL_TERMINATE, conn_id);
    return true;
}

static bc2_status_t g_simple(fake_env_t *env, fake_gcall_kind_t kind, uint8_t conn_id, uint16_t handle,
                             const uint8_t *data, uint8_t len, uint16_t *rc)
{
    fake_gcall_t *c = grecord(env, kind, conn_id);
    c->handle = handle;
    c->len = len;
    if (len > 0) {
        memcpy(c->data, data, len);
    }
    *rc = env->g_rc;
    return env->gresult[kind];
}

static bc2_status_t g_discover(void *ctx, uint8_t conn_id, uint16_t *rc)
{
    return g_simple(ctx, GCALL_DISCOVER, conn_id, 0, NULL, 0, rc);
}

static bc2_status_t g_read(void *ctx, uint8_t conn_id, uint16_t handle, uint16_t *rc)
{
    return g_simple(ctx, GCALL_READ, conn_id, handle, NULL, 0, rc);
}

static bc2_status_t g_write(void *ctx, uint8_t conn_id, uint16_t handle, const uint8_t *data, uint8_t len,
                            uint16_t *rc)
{
    return g_simple(ctx, GCALL_WRITE, conn_id, handle, data, len, rc);
}

static bc2_status_t g_write_cmd(void *ctx, uint8_t conn_id, uint16_t handle, const uint8_t *data, uint8_t len,
                                uint16_t *rc)
{
    fake_env_t *env = ctx;
    if (env->write_cmd_busy > 0) {                        /* controller buffer full: refuse, do not record */
        env->write_cmd_busy--;
        return BC2_ERR_BUSY;
    }
    return g_simple(env, GCALL_WRITE_CMD, conn_id, handle, data, len, rc);
}

static bc2_status_t g_subscribe(void *ctx, uint8_t conn_id, uint16_t cccd, uint8_t mode, uint16_t *rc)
{
    fake_env_t *env = ctx;
    const bc2_status_t st = g_simple(env, GCALL_SUBSCRIBE, conn_id, cccd, NULL, 0, rc);
    env->gcalls[env->gcall_count - 1].mode = mode;
    return st;
}

static bool g_poll_event(void *ctx, bc2_gatt_event_t *ev)
{
    fake_env_t *env = ctx;
    if (env->gev_head == env->gev_tail) {
        return false;
    }
    *ev = env->gev[env->gev_head++ % FAKE_MAX_GEVENTS];
    return true;
}

static void g_stats(void *ctx, bc2_gatt_adapter_stats_t *out)
{
    fake_env_t *env = ctx;
    *out = env->gstats;
    env->gstats.dropped_scan = 0;                         /* deltas are consumed by the call */
    env->gstats.dropped_notify = 0;
    env->gstats.state_lost = false;
}

/* ---- harness helpers ---- */

void fake_init_conns(fake_env_t *env, uint8_t max_conns)
{
    fake_init_no_gatt(env);
    env->gatt_attached = true;
    env->max_conns = max_conns;
    const bc2_ble_ops_t ble = {.ctx = env, .ready = f_ready, .max_sets = f_max_sets,
                               .device_id = f_device_id, .start = f_start, .update = f_update,
                               .stop = f_stop, .take_failure = f_take_failure};
    const bc2_gatt_ops_t gatt = {.ctx = env, .max_conns = g_max_conns, .scan_start = g_scan_start,
                                 .scan_stop = g_scan_stop, .connect = g_connect,
                                 .connect_cancel = g_connect_cancel, .terminate = g_terminate,
                                 .discover = g_discover, .read = g_read, .write = g_write,
                                 .write_cmd = g_write_cmd, .subscribe = g_subscribe,
                                 .poll_event = g_poll_event, .stats = g_stats};
    const bc2_io_ops_t io = {.ctx = env, .write = f_write, .write_droppable = f_write_droppable,
                             .now_us = f_now_us, .hard_reset = f_hard_reset};
    bc2_core_init(&env->core, &ble, &gatt, &io);
}

void fake_init(fake_env_t *env)
{
    fake_init_conns(env, 4);
}

void fake_init_no_gatt(fake_env_t *env)
{
    memset(env, 0, sizeof(*env));
    env->ready = true;
    env->max_sets = 5;
    env->seq = 1;
    const bc2_ble_ops_t ble = {.ctx = env, .ready = f_ready, .max_sets = f_max_sets,
                               .device_id = f_device_id, .start = f_start, .update = f_update,
                               .stop = f_stop, .take_failure = f_take_failure};
    const bc2_io_ops_t io = {.ctx = env, .write = f_write, .now_us = f_now_us,
                             .hard_reset = f_hard_reset};
    bc2_core_init(&env->core, &ble, NULL, &io);
}

void fake_tick(fake_env_t *env, uint32_t now_ms)
{
    env->now_ms = now_ms;
    bc2_core_tick(&env->core, now_ms);
}

void fake_push_event(fake_env_t *env, const bc2_gatt_event_t *ev)
{
    env->gev[env->gev_tail++ % FAKE_MAX_GEVENTS] = *ev;
}

void fake_ev_link_up(fake_env_t *env, uint8_t conn_id, uint16_t mtu, uint16_t itvl)
{
    bc2_gatt_event_t ev = {.kind = BC2_GEV_LINK_UP, .conn_id = conn_id, .u16a = mtu, .u16b = itvl};
    for (int i = 0; i < 6; i++) {
        ev.addr[i] = (uint8_t)(0x10 * (conn_id + 1) + i);   /* recognisable per link */
    }
    fake_push_event(env, &ev);
}

void fake_ev_link_down(fake_env_t *env, uint8_t conn_id, uint16_t reason)
{
    const bc2_gatt_event_t ev = {.kind = BC2_GEV_LINK_DOWN, .conn_id = conn_id, .code = reason};
    fake_push_event(env, &ev);
}

void fake_ev_connect_failed(fake_env_t *env, uint8_t conn_id, uint16_t reason)
{
    const bc2_gatt_event_t ev = {.kind = BC2_GEV_CONNECT_FAILED, .conn_id = conn_id, .code = reason};
    fake_push_event(env, &ev);
}

void fake_ev_op_done(fake_env_t *env, uint8_t conn_id, uint8_t status, uint16_t code, const uint8_t *data, uint8_t len)
{
    bc2_gatt_event_t ev = {.kind = BC2_GEV_OP_DONE, .conn_id = conn_id, .status = status, .code = code, .len = len};
    if (len > 0) {
        memcpy(ev.data, data, len);
    }
    fake_push_event(env, &ev);
}

void fake_ev_notify(fake_env_t *env, uint8_t conn_id, uint16_t handle, const uint8_t *data, uint8_t len)
{
    bc2_gatt_event_t ev = {.kind = BC2_GEV_NOTIFY, .conn_id = conn_id, .u16a = handle, .len = len};
    memcpy(ev.data, data, len);
    fake_push_event(env, &ev);
}

int fake_count_gcalls(const fake_env_t *env, fake_gcall_kind_t kind, uint8_t conn_id)
{
    int n = 0;
    for (int i = 0; i < env->gcall_count; i++) {
        if (env->gcalls[i].kind == kind && (conn_id == 0xFF || env->gcalls[i].conn_id == conn_id)) {
            n++;
        }
    }
    return n;
}

const fake_gcall_t *fake_last_gcall(const fake_env_t *env, fake_gcall_kind_t kind)
{
    for (int i = env->gcall_count - 1; i >= 0; i--) {
        if (env->gcalls[i].kind == kind) {
            return &env->gcalls[i];
        }
    }
    return NULL;
}

void fake_send(fake_env_t *env, uint8_t type, const uint8_t *payload, uint8_t len)
{
    bc2_frame_t fr = {.type = type, .seq = env->seq++, .len = len};
    uint8_t wire[BC2_MAX_FRAME];
    if (len > 0) {
        memcpy(fr.payload, payload, len);
    }
    const size_t n = bc2_frame_encode(&fr, wire, sizeof(wire));
    bc2_core_on_bytes(&env->core, wire, n, env->now_ms);
}

void fake_hello(fake_env_t *env, uint16_t keepalive_ms)
{
    const uint8_t p[4] = {BC2_PROTO_MAJOR, 0, (uint8_t)(keepalive_ms & 0xFFu), (uint8_t)(keepalive_ms >> 8)};
    fake_send(env, BC2_MSG_HELLO, p, sizeof(p));
}

void fake_start(fake_env_t *env, uint8_t set_id, uint16_t mfr, const uint8_t *data, uint8_t len)
{
    uint8_t p[BC2_MAX_PAYLOAD] = {set_id, 0x01, 32, 0, 127, (uint8_t)(mfr & 0xFFu), (uint8_t)(mfr >> 8), len};
    memcpy(&p[8], data, len);
    fake_send(env, BC2_MSG_SET_START, p, (uint8_t)(8u + len));
}

int fake_out_count(const fake_env_t *env)
{
    int n = 0;
    while (fake_out_frame(env, n, &(bc2_frame_t){0})) {
        n++;
    }
    return n;
}

bool fake_out_frame(const fake_env_t *env, int index, bc2_frame_t *fr)
{
    bc2_framer_t f;
    bc2_framer_init(&f);
    int found = 0;
    for (size_t i = 0; i < env->out_len; i++) {
        bc2_framer_push(&f, env->out[i], 0);
        if (bc2_framer_next(&f, fr)) {
            if (found == index) {
                return true;
            }
            found++;
        }
    }
    return false;
}

void fake_clear(fake_env_t *env)
{
    env->out_len = 0;
    env->call_count = 0;
    env->gcall_count = 0;
}

int fake_count_calls(const fake_env_t *env, fake_call_kind_t kind)
{
    int n = 0;
    for (int i = 0; i < env->call_count; i++) {
        if (env->calls[i].kind == kind) {
            n++;
        }
    }
    return n;
}
