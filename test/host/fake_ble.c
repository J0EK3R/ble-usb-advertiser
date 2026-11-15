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

/* ---- harness helpers ---- */

void fake_init(fake_env_t *env)
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
    bc2_core_init(&env->core, &ble, &io);
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
