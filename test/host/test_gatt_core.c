/**
 * @file test_gatt_core.c
 * @brief Tests of the GATT central logic in bc2_gatt / bc2_core with a scriptable fake adapter:
 *        request validation order, deferred ACKs, connect scheduling, discovery, procedures,
 *        timeouts, notifications, failsafe writes and the safety stop (keep-alive, button, HELLO).
 *
 * Most tests are parametrised over max_conns (1, 4, 9, 16) where the limit matters, so nothing
 * depends on the value that the hardware spike measures.
 */
#include "bc2_core.h"
#include "fake_ble.h"
#include "test_minunit.h"

/** Assert that output frame @p idx is an ACK for @p orig with @p status. */
#define ASSERT_ACK(env, idx, orig, status)                                       \
    do {                                                                         \
        bc2_frame_t f_;                                                          \
        ASSERT_TRUE(fake_out_frame((env), (idx), &f_));                          \
        ASSERT_EQ(BC2_MSG_ACK, f_.type);                                         \
        ASSERT_EQ((orig), f_.payload[0]);                                        \
        ASSERT_EQ((status), f_.payload[1]);                                      \
    } while (0)

/** Assert that output frame @p idx has TYPE @p t. */
#define ASSERT_TYPE(env, idx, t)                                                 \
    do {                                                                         \
        bc2_frame_t f_;                                                          \
        ASSERT_TRUE(fake_out_frame((env), (idx), &f_));                          \
        ASSERT_EQ((t), f_.type);                                                 \
    } while (0)

static const uint8_t MAXES[] = {1, 4, 9, 16};

/* ---- helpers ---- */

/** Open a session with a long keep-alive so that tests do not time out by accident. */
static void open_session(fake_env_t *env)
{
    fake_hello(env, 10000);
    fake_clear(env);
}

/** Send CONNECT for @p id to the address xx:xx:xx:xx:xx:@p dev. */
static void send_connect(fake_env_t *env, uint8_t id, uint8_t dev, uint8_t flags, uint16_t timeout_ms)
{
    const uint8_t p[15] = {id, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, dev, BC2_ADDR_TYPE_AUTO, flags,
                           (uint8_t)(timeout_ms & 0xFFu), (uint8_t)(timeout_ms >> 8), 0, 0, 0, 0};
    fake_send(env, BC2_MSG_CONNECT, p, sizeof(p));
}

/** Bring link @p id to the connected state with MTU @p mtu and clear the captured output. */
static void make_connected(fake_env_t *env, uint8_t id, uint16_t mtu)
{
    send_connect(env, id, (uint8_t)(0x40 + id), 0, 0);
    fake_ev_link_up(env, id, mtu, 24);
    fake_tick(env, env->now_ms);
    fake_clear(env);
}

static void send_conn_id_req(fake_env_t *env, uint8_t type, uint8_t id)
{
    fake_send(env, type, &id, 1);
}

static void send_rw(fake_env_t *env, uint8_t type, uint8_t id, uint16_t handle, const uint8_t *data, uint8_t len)
{
    uint8_t p[BC2_MAX_PAYLOAD] = {id, (uint8_t)(handle & 0xFFu), (uint8_t)(handle >> 8)};
    if (len > 0) {
        memcpy(&p[3], data, len);
    }
    fake_send(env, type, p, (uint8_t)(3u + len));
}

static void send_subscribe(fake_env_t *env, uint8_t id, uint16_t cccd, uint8_t mode)
{
    const uint8_t p[4] = {id, (uint8_t)(cccd & 0xFFu), (uint8_t)(cccd >> 8), mode};
    fake_send(env, BC2_MSG_SUBSCRIBE, p, sizeof(p));
}

static void send_failsafe(fake_env_t *env, uint8_t id, uint8_t slot, uint16_t handle, const uint8_t *data, uint8_t len)
{
    uint8_t p[BC2_MAX_PAYLOAD] = {id, slot, (uint8_t)(handle & 0xFFu), (uint8_t)(handle >> 8)};
    if (len > 0) {
        memcpy(&p[4], data, len);
    }
    fake_send(env, BC2_MSG_SET_FAILSAFE, p, (uint8_t)(4u + len));
}

/* ---- capabilities and basic dispatch ---- */

/** HELLO_RSP announces the capability block; max_conns follows the adapter (capped at 16). */
static void test_hello_capabilities(void)
{
    for (size_t i = 0; i < sizeof(MAXES); i++) {
        fake_env_t env;
        bc2_frame_t fr;
        fake_init_conns(&env, MAXES[i]);
        fake_hello(&env, 1000);
        ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
        ASSERT_EQ(32, fr.len);
        ASSERT_EQ(1, fr.payload[8]);                       /* proto minor */
        ASSERT_EQ(2, fr.payload[10]);                      /* fw minor 0.2.0 */
        ASSERT_EQ(7, fr.payload[27]);                      /* scan | central | failsafe */
        ASSERT_EQ(MAXES[i], fr.payload[28]);
        ASSERT_EQ(56, fr.payload[29]);
        ASSERT_EQ(31, fr.payload[30]);
        ASSERT_EQ(4, fr.payload[31]);
    }
    fake_env_t env;
    bc2_frame_t fr;
    fake_init_conns(&env, 40);                             /* adapter claims more than the protocol allows */
    fake_hello(&env, 1000);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(16, fr.payload[28]);
}

/** A core built without GATT announces nothing and answers the GATT requests NOT_SUPPORTED. */
static void test_no_gatt_build(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init_no_gatt(&env);
    fake_hello(&env, 1000);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(0, fr.payload[27]);
    ASSERT_EQ(0, fr.payload[28]);
    ASSERT_EQ(0, fr.payload[31]);
    fake_clear(&env);
    for (uint8_t t = BC2_MSG_SCAN_START; t <= BC2_MSG_SET_FAILSAFE; t++) {
        fake_send(&env, t, NULL, 0);
    }
    for (int i = 0; i < 11; i++) {
        ASSERT_ACK(&env, i, BC2_MSG_SCAN_START + i, BC2_ERR_NOT_SUPPORTED);
    }
    ASSERT_TRUE(!bc2_core_gatt_active(&env.core));
}

/** Without a session every GATT request answers NO_SESSION, before any parameter check. */
static void test_gatt_needs_session(void)
{
    fake_env_t env;
    fake_init(&env);
    for (uint8_t t = BC2_MSG_SCAN_START; t <= BC2_MSG_SET_FAILSAFE; t++) {
        fake_send(&env, t, NULL, 0);
    }
    for (int i = 0; i < 11; i++) {
        ASSERT_ACK(&env, i, BC2_MSG_SCAN_START + i, BC2_ERR_NO_SESSION);
    }
    ASSERT_EQ(0, env.gcall_count);
}

/** Wrong payload lengths are BAD_LENGTH for every request (check order: length first). */
static void test_gatt_bad_length(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    const uint8_t junk[3] = {0, 0, 0};
    for (uint8_t t = BC2_MSG_SCAN_START; t <= BC2_MSG_SET_FAILSAFE; t++) {
        fake_clear(&env);
        const uint8_t len = (t == BC2_MSG_CONNECT || t == BC2_MSG_SCAN_START || t == BC2_MSG_READ ||
                             t == BC2_MSG_SUBSCRIBE || t == BC2_MSG_SET_FAILSAFE || t == BC2_MSG_WRITE ||
                             t == BC2_MSG_WRITE_CMD) ? 2u : 3u;   /* never the right size for any request */
        fake_send(&env, t, junk, len);
        ASSERT_ACK(&env, 0, t, BC2_ERR_BAD_LENGTH);
    }
    ASSERT_EQ(0, env.gcall_count);
}

/* ---- scanning ---- */

static void test_scan_start_stop_and_reports(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    const uint8_t p[7] = {0x01, 0, 0, 0, 0, 0, 0};
    fake_send(&env, BC2_MSG_SCAN_START, p, sizeof(p));
    ASSERT_ACK(&env, 0, BC2_MSG_SCAN_START, BC2_OK);
    const fake_gcall_t *c = fake_last_gcall(&env, GCALL_SCAN_START);
    ASSERT_TRUE(c != NULL);
    ASSERT_EQ(1, c->mode);
    ASSERT_EQ(160, c->itvl_min);                           /* defaults applied by the decoder */
    ASSERT_EQ(80, c->itvl_max);
    ASSERT_EQ(1000, c->handle);
    ASSERT_TRUE(bc2_core_gatt_active(&env.core));
    ASSERT_EQ(BC2_LED_ADVERTISING, bc2_core_led_state(&env.core, 0));

    fake_send(&env, BC2_MSG_SCAN_START, p, sizeof(p));     /* second start: stop first */
    ASSERT_ACK(&env, 1, BC2_MSG_SCAN_START, BC2_ERR_ALREADY_RUNNING);

    bc2_gatt_event_t ev = {.kind = BC2_GEV_SCAN_REPORT, .addr_type = 1, .rssi = -60, .flags = 1, .len = 3,
                           .addr = {1, 2, 3, 4, 5, 6}, .data = {2, 1, 6}};
    fake_clear(&env);
    fake_push_event(&env, &ev);
    fake_tick(&env, 10);
    ASSERT_TYPE(&env, 0, BC2_MSG_EVT_SCAN_REPORT);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(13, fr.len);
    ASSERT_EQ(0xC4, fr.payload[7]);                        /* rssi -60 */

    fake_clear(&env);
    fake_send(&env, BC2_MSG_SCAN_STOP, NULL, 0);
    ASSERT_ACK(&env, 0, BC2_MSG_SCAN_STOP, BC2_OK);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_SCAN_STOP, 0xFF));
    fake_clear(&env);
    fake_push_event(&env, &ev);                            /* late report after the stop: ignored */
    fake_tick(&env, 20);
    ASSERT_EQ(0, fake_out_count(&env));
    fake_send(&env, BC2_MSG_SCAN_STOP, NULL, 0);           /* always OK, also when not scanning */
    ASSERT_ACK(&env, 0, BC2_MSG_SCAN_STOP, BC2_OK);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_SCAN_STOP, 0xFF));   /* nothing to stop: no adapter call */
}

static void test_scan_errors(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    const uint8_t bad[7] = {0x04, 0, 0, 0, 0, 0, 0};       /* reserved flag bit */
    fake_send(&env, BC2_MSG_SCAN_START, bad, sizeof(bad));
    ASSERT_ACK(&env, 0, BC2_MSG_SCAN_START, BC2_ERR_BAD_PARAM);
    env.gresult[GCALL_SCAN_START] = BC2_ERR_BLE_ERROR;
    env.g_rc = 0x0102;
    const uint8_t ok[7] = {0x01, 0, 0, 0, 0, 0, 0};
    fake_send(&env, BC2_MSG_SCAN_START, ok, sizeof(ok));
    bc2_frame_t fr;
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_ERR_BLE_ERROR, fr.payload[1]);
    ASSERT_EQ(0x0102, fr.payload[2] | (fr.payload[3] << 8));   /* rc travels as ACK extra */
    ASSERT_TRUE(!bc2_core_gatt_active(&env.core));         /* a failed start does not count as scanning */
}

/** The adapter reports a scan that stopped by itself. */
static void test_scan_stopped_by_adapter(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    const uint8_t ok[7] = {0x01, 0, 0, 0, 0, 0, 0};
    fake_send(&env, BC2_MSG_SCAN_START, ok, sizeof(ok));
    fake_clear(&env);
    const bc2_gatt_event_t ev = {.kind = BC2_GEV_SCAN_STOPPED, .code = 15};
    fake_push_event(&env, &ev);
    fake_tick(&env, 5);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_SCAN_STOPPED, fr.type);
    ASSERT_EQ(BC2_SCAN_STOP_ERROR, fr.payload[0]);
    ASSERT_EQ(15, fr.payload[1]);
    ASSERT_TRUE(!bc2_core_gatt_active(&env.core));
    fake_send(&env, BC2_MSG_SCAN_START, ok, sizeof(ok));   /* can be restarted */
    ASSERT_ACK(&env, 1, BC2_MSG_SCAN_START, BC2_OK);
}

/* ---- connecting ---- */

static void test_connect_flow(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    env.now_ms = 100;
    send_connect(&env, 2, 0x55, 0, 5000);
    ASSERT_ACK(&env, 0, BC2_MSG_CONNECT, BC2_OK);
    const fake_gcall_t *c = fake_last_gcall(&env, GCALL_CONNECT);
    ASSERT_TRUE(c != NULL);
    ASSERT_EQ(2, c->conn_id);
    ASSERT_EQ(0x55, c->addr[5]);
    ASSERT_EQ(BC2_ADDR_TYPE_AUTO, c->mode);                /* auto is resolved by the adapter */
    ASSERT_EQ(12, c->itvl_min);
    ASSERT_EQ(24, c->itvl_max);
    ASSERT_EQ(5000, c->duration_ms);
    ASSERT_EQ(1, fake_out_count(&env));                    /* only the ACK so far */

    fake_send(&env, BC2_MSG_GET_GATT_STATUS, NULL, 0);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(4, fr.payload[1]);                           /* connecting_mask: conn 2 */

    fake_ev_link_up(&env, 2, 64, 24);
    fake_tick(&env, 150);
    ASSERT_TYPE(&env, 2, BC2_MSG_EVT_CONNECTED);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(2, fr.payload[0]);
    ASSERT_EQ(64, fr.payload[1] | (fr.payload[2] << 8));
    ASSERT_EQ(24, fr.payload[3] | (fr.payload[4] << 8));
    ASSERT_EQ(0x30, fr.payload[5]);                        /* address of the fake link */
    ASSERT_EQ(BC2_LED_ADVERTISING, bc2_core_led_state(&env.core, 150));
    fake_send(&env, BC2_MSG_GET_GATT_STATUS, NULL, 0);
    ASSERT_TRUE(fake_out_frame(&env, 3, &fr));
    ASSERT_EQ(0, fr.payload[1]);
    ASSERT_EQ(4, fr.payload[3]);                           /* connected_mask */
}

/** MTU exchange failed on the adapter side: the link is still reported, with the default MTU 23. */
static void test_connect_with_default_mtu(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, 0, 0);
    fake_ev_link_up(&env, 0, BC2_GATT_DEFAULT_MTU, 40);
    fake_tick(&env, 1);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_CONNECTED, fr.type);
    ASSERT_EQ(23, fr.payload[1]);
    const uint8_t d21[21] = {0};
    fake_clear(&env);
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, d21, 21);    /* 21 + 3 > mtu 23 */
    ASSERT_ACK(&env, 0, BC2_MSG_WRITE_CMD, BC2_ERR_PAYLOAD_TOO_LARGE);
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, d21, 20);    /* 20 + 3 = mtu 23: fits */
    ASSERT_ACK(&env, 1, BC2_MSG_WRITE_CMD, BC2_OK);
}

/** The slot limit follows max_conns: one more connection answers NO_RESOURCES. */
static void test_connect_limit(void)
{
    for (size_t k = 0; k < sizeof(MAXES); k++) {
        fake_env_t env;
        fake_init_conns(&env, MAXES[k]);
        open_session(&env);
        for (uint8_t i = 0; i < MAXES[k]; i++) {
            send_connect(&env, i, (uint8_t)(0x10 + i), BC2_CONNECT_FLAG_BACKGROUND, 0);
            ASSERT_ACK(&env, i, BC2_MSG_CONNECT, BC2_OK);
        }
        send_connect(&env, MAXES[k] == 16 ? 0 : MAXES[k], 0x99, 0, 0);   /* a free conn_id, but no free link */
        if (MAXES[k] == 16) {
            ASSERT_ACK(&env, MAXES[k], BC2_MSG_CONNECT, BC2_ERR_ALREADY_RUNNING);   /* id 0 is in use */
        } else {
            ASSERT_ACK(&env, MAXES[k], BC2_MSG_CONNECT, BC2_ERR_NO_RESOURCES);
        }
    }
}

static void test_connect_errors(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 16, 1, 0, 0);                       /* conn_id out of range */
    ASSERT_ACK(&env, 0, BC2_MSG_CONNECT, BC2_ERR_BAD_PARAM);
    send_connect(&env, 1, 7, 0, 0);
    ASSERT_ACK(&env, 1, BC2_MSG_CONNECT, BC2_OK);
    send_connect(&env, 1, 8, 0, 0);                        /* id in use */
    ASSERT_ACK(&env, 2, BC2_MSG_CONNECT, BC2_ERR_ALREADY_RUNNING);
    send_connect(&env, 2, 7, 0, 0);                        /* same address under another id */
    ASSERT_ACK(&env, 3, BC2_MSG_CONNECT, BC2_ERR_ALREADY_RUNNING);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
}

/** The adapter refuses to start an attempt: EVT_DISCONNECTED (phase 0, reason 0xF3), slot freed. */
static void test_connect_adapter_error(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    env.gresult[GCALL_CONNECT] = BC2_ERR_NO_RESOURCES;
    send_connect(&env, 0, 1, 0, 0);
    ASSERT_ACK(&env, 0, BC2_MSG_CONNECT, BC2_OK);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_PHASE_CONNECTING, fr.payload[1]);
    ASSERT_EQ(BC2_REASON_INTERNAL, fr.payload[2]);
    env.gresult[GCALL_CONNECT] = BC2_OK;
    send_connect(&env, 0, 1, 0, 0);                        /* id free again */
    ASSERT_ACK(&env, 2, BC2_MSG_CONNECT, BC2_OK);
}

/** One initiator: the second CONNECT waits until the first attempt has ended. */
static void test_connect_serialised(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, 0, 0);
    send_connect(&env, 1, 2, 0, 0);
    send_connect(&env, 2, 3, 0, 0);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
    fake_ev_link_up(&env, 0, 64, 24);
    fake_tick(&env, 10);
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
    ASSERT_EQ(1, fake_last_gcall(&env, GCALL_CONNECT)->conn_id);
    fake_ev_link_up(&env, 1, 64, 24);
    fake_tick(&env, 20);
    ASSERT_EQ(3, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
    ASSERT_EQ(2, fake_last_gcall(&env, GCALL_CONNECT)->conn_id);
}

/** A background connect yields after the 3 s slice when another one waits (round robin). */
static void test_connect_background_slice(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, BC2_CONNECT_FLAG_BACKGROUND, 0);   /* the DUPLO that is not switched on yet */
    send_connect(&env, 1, 2, 0, 20000);
    ASSERT_EQ(0, fake_last_gcall(&env, GCALL_CONNECT)->duration_ms);   /* background: no duration */
    fake_tick(&env, 2999);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
    fake_tick(&env, 3000);                                 /* slice over, another connect is due */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
    fake_ev_connect_failed(&env, 0, 0x02);
    fake_tick(&env, 3010);                                 /* conn 1 gets the initiator, conn 0 waits */
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
    ASSERT_EQ(1, fake_last_gcall(&env, GCALL_CONNECT)->conn_id);
    ASSERT_EQ(20000 - 3010, fake_last_gcall(&env, GCALL_CONNECT)->duration_ms);
    fake_ev_link_up(&env, 1, 64, 24);
    fake_tick(&env, 3020);
    ASSERT_EQ(3, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));   /* background connect is tried again */
    ASSERT_EQ(0, fake_last_gcall(&env, GCALL_CONNECT)->conn_id);
}

/** Without waiting competitors the attempt is never cancelled. */
static void test_connect_no_slice_when_alone(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, BC2_CONNECT_FLAG_BACKGROUND, 0);
    fake_tick(&env, 9000);                                 /* well past the 3 s slice, before the keep-alive */
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
}

/** Timeout of a waiting request and of a running attempt: EVT_DISCONNECTED phase 0, reason 0xF0. */
static void test_connect_timeout(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, 0, 1000);                     /* running attempt */
    send_connect(&env, 1, 2, 0, 1000);                     /* waiting behind it */
    fake_clear(&env);
    fake_tick(&env, 999);
    ASSERT_EQ(0, fake_out_count(&env));
    fake_tick(&env, 1000);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));             /* the waiting one ends at once */
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(1, fr.payload[0]);
    ASSERT_EQ(BC2_REASON_CONNECT_TIMEOUT, fr.payload[2]);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));   /* the running one is cancelled */
    fake_ev_connect_failed(&env, 0, 0x02);
    fake_tick(&env, 1010);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(0, fr.payload[0]);
    ASSERT_EQ(BC2_PHASE_CONNECTING, fr.payload[1]);
    ASSERT_EQ(BC2_REASON_CONNECT_TIMEOUT, fr.payload[2]);
}

/** A failed attempt is retried after 200 ms until the deadline, then the host hears 0xF0. */
static void test_connect_retry_until_deadline(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, 0, 1000);
    fake_ev_connect_failed(&env, 0, 0x3E);
    fake_tick(&env, 100);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
    fake_tick(&env, 300);                                  /* 200 ms later: second attempt */
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
    fake_clear(&env);
    fake_ev_connect_failed(&env, 0, 0x3E);
    fake_tick(&env, 1000);                                 /* failure at the deadline: give up */
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_REASON_CONNECT_TIMEOUT, fr.payload[2]);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));
}

/** The attempt does not answer to the cancel request: the guard timer ends it. */
static void test_connect_cancel_guard(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, 0, 0);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 0);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
    fake_clear(&env);
    fake_tick(&env, BC2_GATT_CANCEL_GUARD_MS - 1);
    ASSERT_EQ(0, fake_out_count(&env));
    fake_tick(&env, BC2_GATT_CANCEL_GUARD_MS);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_REASON_CANCELLED, fr.payload[2]);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));   /* a late link must not survive */
}

/* ---- disconnecting ---- */

static void test_disconnect_variants(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 5);         /* free id: OK, no event */
    ASSERT_ACK(&env, 0, BC2_MSG_DISCONNECT, BC2_OK);
    ASSERT_EQ(1, fake_out_count(&env));

    fake_clear(&env);
    send_connect(&env, 0, 1, 0, 0);                        /* running attempt */
    send_connect(&env, 1, 2, 0, 0);                        /* waiting */
    fake_clear(&env);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 1);         /* waiting: cancelled at once */
    ASSERT_ACK(&env, 0, BC2_MSG_DISCONNECT, BC2_OK);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_REASON_CANCELLED, fr.payload[2]);
    fake_clear(&env);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 0);         /* running: cancel, event after the adapter answers */
    ASSERT_EQ(1, fake_out_count(&env));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
    fake_ev_connect_failed(&env, 0, 0x02);
    fake_tick(&env, 5);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_REASON_CANCELLED, fr.payload[2]);
    ASSERT_EQ(BC2_PHASE_CONNECTING, fr.payload[1]);

    make_connected(&env, 3, 64);                           /* connected: terminate, event with the stack's reason */
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 3);
    ASSERT_ACK(&env, 0, BC2_MSG_DISCONNECT, BC2_OK);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 3));
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 3);         /* again while terminating: OK, no second terminate */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 3));
    fake_ev_link_down(&env, 3, 0x16);
    fake_tick(&env, 10);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_PHASE_CONNECTED, fr.payload[1]);
    ASSERT_EQ(0x16, fr.payload[2]);
    send_connect(&env, 3, 9, 0, 0);                        /* the id is free again */
    ASSERT_ACK(&env, 3, BC2_MSG_CONNECT, BC2_OK);
}

/** A link that nobody asked for (cancel lost the race, stale attempt) is terminated, never kept. */
static void test_orphan_and_raced_links(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    fake_ev_link_up(&env, 7, 64, 24);                      /* no such attempt */
    fake_tick(&env, 1);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 7));
    ASSERT_EQ(0, fake_out_count(&env));                    /* never announced */

    send_connect(&env, 0, 1, 0, 0);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 0);         /* cancel is on its way ... */
    fake_clear(&env);
    fake_ev_link_up(&env, 0, 64, 24);                      /* ... but the link came up first */
    fake_tick(&env, 2);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    ASSERT_EQ(0, fake_out_count(&env));                    /* no EVT_CONNECTED for a cancelled request */
    fake_ev_link_down(&env, 0, 0x16);
    fake_tick(&env, 3);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_PHASE_CONNECTING, fr.payload[1]);        /* the host never saw the link */
    ASSERT_EQ(BC2_REASON_CANCELLED, fr.payload[2]);
}

/** Link lost while connecting (e.g. during the MTU exchange) counts as a failed attempt. */
static void test_link_down_during_attempt(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    send_connect(&env, 0, 1, 0, 10000);
    fake_ev_link_down(&env, 0, 0x3E);
    fake_tick(&env, 500);
    fake_tick(&env, 800);
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_CONNECT, 0xFF));   /* retried */
}

/* ---- discovery ---- */

static void push_service(fake_env_t *env, uint8_t id, uint16_t start, uint16_t end)
{
    bc2_gatt_event_t ev = {.kind = BC2_GEV_SERVICE, .conn_id = id, .u16a = start, .u16b = end, .len = 2,
                           .data = {0x0A, 0x18}};
    fake_push_event(env, &ev);
}

static void push_char(fake_env_t *env, uint8_t id, uint16_t value, uint16_t cccd)
{
    bc2_gatt_event_t ev = {.kind = BC2_GEV_CHAR, .conn_id = id, .u16a = value, .u16b = cccd, .code = 0x1E,
                           .len = 2, .data = {0x26, 0x2A}};
    fake_push_event(env, &ev);
}

static void test_discovery(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 1, 64);
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 1);
    ASSERT_ACK(&env, 0, BC2_MSG_DISCOVER, BC2_OK);
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 1);           /* running: BUSY */
    ASSERT_ACK(&env, 1, BC2_MSG_DISCOVER, BC2_ERR_BUSY);
    const uint8_t b = 1;
    send_rw(&env, BC2_MSG_READ, 1, 0x10, &b, 0);
    ASSERT_ACK(&env, 2, BC2_MSG_READ, BC2_ERR_BUSY);       /* any procedure is BUSY meanwhile */
    push_service(&env, 1, 0x10, 0x20);
    push_service(&env, 1, 0x30, 0x3F);
    push_char(&env, 1, 0x12, 0x13);
    push_char(&env, 1, 0x32, 0);
    push_char(&env, 1, 0x34, 0);
    const bc2_gatt_event_t done = {.kind = BC2_GEV_DISCOVER_DONE, .conn_id = 1, .status = BC2_OK};
    fake_push_event(&env, &done);
    fake_clear(&env);
    fake_tick(&env, 100);
    ASSERT_TYPE(&env, 0, BC2_MSG_EVT_GATT_SERVICE);
    ASSERT_TYPE(&env, 1, BC2_MSG_EVT_GATT_SERVICE);
    ASSERT_TYPE(&env, 2, BC2_MSG_EVT_GATT_CHAR);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(0x13, fr.payload[3]);                        /* CCCD handle */
    ASSERT_TYPE(&env, 5, BC2_MSG_EVT_DISCOVER_DONE);
    ASSERT_TRUE(fake_out_frame(&env, 5, &fr));
    ASSERT_EQ(BC2_OK, fr.payload[1]);
    ASSERT_EQ(2, fr.payload[2]);
    ASSERT_EQ(3, fr.payload[3]);
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 1);           /* free for the next procedure */
    ASSERT_ACK(&env, 6, BC2_MSG_DISCOVER, BC2_OK);
}

static void test_discovery_errors(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 2);           /* nothing connected */
    ASSERT_ACK(&env, 0, BC2_MSG_DISCOVER, BC2_ERR_NOT_CONNECTED);
    make_connected(&env, 2, 64);
    env.gresult[GCALL_DISCOVER] = BC2_ERR_NO_RESOURCES;
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 2);
    ASSERT_ACK(&env, 0, BC2_MSG_DISCOVER, BC2_ERR_NO_RESOURCES);
    env.gresult[GCALL_DISCOVER] = BC2_OK;

    /* Link lost during discovery: DISCOVER_DONE(DISCONNECTED) comes before EVT_DISCONNECTED. */
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 2);
    push_service(&env, 2, 1, 5);
    fake_ev_link_down(&env, 2, 0x08);
    fake_clear(&env);
    fake_tick(&env, 5);
    ASSERT_TYPE(&env, 0, BC2_MSG_EVT_GATT_SERVICE);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCOVER_DONE, fr.type);
    ASSERT_EQ(BC2_ERR_DISCONNECTED, fr.payload[1]);
    ASSERT_EQ(1, fr.payload[2]);
    ASSERT_TYPE(&env, 2, BC2_MSG_EVT_DISCONNECTED);
}

/** The 5 s discovery timeout restarts with every event; silence ends it with TIMEOUT and a dropped link. */
static void test_discovery_timeout(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    env.now_ms = 1000;
    send_conn_id_req(&env, BC2_MSG_DISCOVER, 0);
    push_service(&env, 0, 1, 5);
    fake_tick(&env, 5000);                                 /* event at 5.0 s restarts the timer */
    fake_clear(&env);
    fake_tick(&env, 9999);
    ASSERT_EQ(0, fake_out_count(&env));
    fake_tick(&env, 10000);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCOVER_DONE, fr.type);
    ASSERT_EQ(BC2_ERR_TIMEOUT, fr.payload[1]);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_ev_link_down(&env, 0, 0x16);
    fake_tick(&env, 10010);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_REASON_INTERNAL, fr.payload[2]);         /* stick specific reason, not the HCI 0x16 */
}

/* ---- read / write / subscribe ---- */

static void test_read(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    send_rw(&env, BC2_MSG_READ, 3, 0x12, NULL, 0);
    ASSERT_ACK(&env, 0, BC2_MSG_READ, BC2_ERR_NOT_CONNECTED);
    make_connected(&env, 3, 64);
    send_rw(&env, BC2_MSG_READ, 3, 0x12, NULL, 0);
    ASSERT_EQ(0, fake_out_count(&env));                    /* answered later */
    const fake_gcall_t *c = fake_last_gcall(&env, GCALL_READ);
    ASSERT_EQ(0x12, c->handle);
    send_rw(&env, BC2_MSG_READ, 3, 0x14, NULL, 0);
    ASSERT_ACK(&env, 0, BC2_MSG_READ, BC2_ERR_BUSY);
    const uint8_t val[2] = {0xA1, 0xB2};
    fake_ev_op_done(&env, 3, BC2_OK, 0, val, 2);
    fake_tick(&env, 5);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_READ_RSP, fr.type);
    ASSERT_EQ(6, fr.len);
    ASSERT_EQ(0x12, fr.payload[1]);
    ASSERT_EQ(0xB2, fr.payload[5]);
    /* READ_RSP carries the SEQ of the request: the first READ was the 2nd request sent */
    bc2_frame_t ack;
    ASSERT_TRUE(fake_out_frame(&env, 0, &ack));
    ASSERT_TRUE(fr.seq != ack.seq);
}

/** ATT errors travel as ACK extra, adapter BUSY and BLE errors are passed on. */
static void test_procedure_errors(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t d[4] = {1, 2, 3, 4};
    send_rw(&env, BC2_MSG_WRITE, 0, 0x2A, d, 4);
    fake_ev_op_done(&env, 0, BC2_ERR_ATT_ERROR, 0x03, NULL, 0);   /* write not permitted */
    fake_tick(&env, 1);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_ACK, fr.type);
    ASSERT_EQ(BC2_MSG_WRITE, fr.payload[0]);
    ASSERT_EQ(BC2_ERR_ATT_ERROR, fr.payload[1]);
    ASSERT_EQ(3, fr.payload[2]);
    ASSERT_EQ(3, fr.len);

    env.gresult[GCALL_READ] = BC2_ERR_BUSY;                /* controller queue full */
    send_rw(&env, BC2_MSG_READ, 0, 0x2A, NULL, 0);
    ASSERT_ACK(&env, 1, BC2_MSG_READ, BC2_ERR_BUSY);
    env.gresult[GCALL_READ] = BC2_ERR_BLE_ERROR;
    env.g_rc = 0x0107;
    send_rw(&env, BC2_MSG_READ, 0, 0x2A, NULL, 0);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(BC2_ERR_BLE_ERROR, fr.payload[1]);
    ASSERT_EQ(0x0107, fr.payload[2] | (fr.payload[3] << 8));
    env.gresult[GCALL_READ] = BC2_OK;
    send_rw(&env, BC2_MSG_READ, 0, 0x2A, NULL, 0);         /* the failed requests did not leave a pending op */
    ASSERT_EQ(3, fake_out_count(&env));
}

/** WRITE with response is ACKed after the ATT response; WRITE_CMD is always allowed and ACKs at once. */
static void test_write_deferred_ack(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t duplo[8] = {0x08, 0x00, 0x81, 0x00, 0x11, 0x51, 0x00, 0x00};
    send_rw(&env, BC2_MSG_WRITE, 0, 0x10, duplo, 8);
    ASSERT_EQ(0, fake_out_count(&env));                    /* waiting for the peer */
    const fake_gcall_t *c = fake_last_gcall(&env, GCALL_WRITE);
    ASSERT_EQ(8, c->len);
    ASSERT_EQ(0x51, c->data[5]);
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, duplo, 8);   /* not a procedure: not BUSY */
    ASSERT_ACK(&env, 0, BC2_MSG_WRITE_CMD, BC2_OK);
    send_rw(&env, BC2_MSG_WRITE, 0, 0x10, duplo, 8);       /* second WRITE: BUSY */
    ASSERT_ACK(&env, 1, BC2_MSG_WRITE, BC2_ERR_BUSY);
    fake_ev_op_done(&env, 0, BC2_OK, 0, NULL, 0);
    fake_tick(&env, 3);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(BC2_MSG_ACK, fr.type);
    ASSERT_EQ(BC2_MSG_WRITE, fr.payload[0]);
    ASSERT_EQ(BC2_OK, fr.payload[1]);
    fake_ev_op_done(&env, 0, BC2_OK, 0, NULL, 0);          /* stray completion: ignored */
    fake_tick(&env, 4);
    ASSERT_EQ(3, fake_out_count(&env));
}

/** WRITE_CMD: BUSY when the controller has no buffer; errors are not procedures. */
static void test_write_cmd_busy(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t d[1] = {1};
    env.gresult[GCALL_WRITE_CMD] = BC2_ERR_BUSY;
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, d, 1);
    ASSERT_ACK(&env, 0, BC2_MSG_WRITE_CMD, BC2_ERR_BUSY);
    env.gresult[GCALL_WRITE_CMD] = BC2_OK;
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, d, 1);
    ASSERT_ACK(&env, 1, BC2_MSG_WRITE_CMD, BC2_OK);
    send_rw(&env, BC2_MSG_WRITE_CMD, 1, 0x10, d, 1);       /* other link is not connected */
    ASSERT_ACK(&env, 2, BC2_MSG_WRITE_CMD, BC2_ERR_NOT_CONNECTED);
}

/** The largest value (56 bytes) goes through when the MTU allows it (BuWizz 3 needs 38+3). */
static void test_write_sizes(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    uint8_t d[57] = {0};
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, d, 56);
    ASSERT_ACK(&env, 0, BC2_MSG_WRITE_CMD, BC2_OK);
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0x10, d, 57);
    ASSERT_ACK(&env, 1, BC2_MSG_WRITE_CMD, BC2_ERR_PAYLOAD_TOO_LARGE);
    send_rw(&env, BC2_MSG_WRITE_CMD, 0, 0, d, 4);          /* handle 0 */
    ASSERT_ACK(&env, 2, BC2_MSG_WRITE_CMD, BC2_ERR_BAD_PARAM);
    ASSERT_EQ(56, fake_last_gcall(&env, GCALL_WRITE_CMD)->len);
}

/** Procedure without answer: TIMEOUT ACK after 5 s, failsafe + terminate, reason 0xF3. */
static void test_procedure_timeout(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    env.now_ms = 2000;
    const uint8_t d[2] = {1, 2};
    send_rw(&env, BC2_MSG_WRITE, 0, 0x10, d, 2);
    send_failsafe(&env, 0, 0, 0x10, d, 2);
    fake_clear(&env);
    fake_tick(&env, 6999);
    ASSERT_EQ(0, fake_out_count(&env));
    fake_tick(&env, 7000);
    ASSERT_ACK(&env, 0, BC2_MSG_WRITE, BC2_ERR_TIMEOUT);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));   /* failsafe write first ... */
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 7000 + BC2_GATT_FAILSAFE_FLUSH_MS);          /* ... terminate after the flush time */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_ev_link_down(&env, 0, 0x16);
    fake_tick(&env, 7100);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_REASON_INTERNAL, fr.payload[2]);
    /* late completion of the timed-out write is ignored */
    fake_ev_op_done(&env, 0, BC2_OK, 0, NULL, 0);
    fake_tick(&env, 7200);
    ASSERT_EQ(2, fake_out_count(&env));
}

/** A request pending while the link is lost is answered DISCONNECTED. */
static void test_pending_on_link_loss(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    send_subscribe(&env, 0, 0x11, BC2_SUB_NOTIFY);
    fake_ev_link_down(&env, 0, 0x08);
    fake_tick(&env, 9);
    ASSERT_ACK(&env, 0, BC2_MSG_SUBSCRIBE, BC2_ERR_DISCONNECTED);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DISCONNECTED, fr.type);
    ASSERT_EQ(BC2_PHASE_CONNECTED, fr.payload[1]);
    ASSERT_EQ(0x08, fr.payload[2]);
}

/**
 * Hubs such as the DUPLO train hub notify right after the CCCD write: the SUBSCRIBE ACK is sent
 * first, then the notifications, in the order in which they happened.
 */
static void test_subscribe_ack_before_notify(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    send_subscribe(&env, 0, 0x11, BC2_SUB_NOTIFY);
    ASSERT_EQ(BC2_SUB_NOTIFY, fake_last_gcall(&env, GCALL_SUBSCRIBE)->mode);
    ASSERT_EQ(0x11, fake_last_gcall(&env, GCALL_SUBSCRIBE)->handle);
    const uint8_t n1[3] = {1, 1, 1}, n2[3] = {2, 2, 2};
    fake_ev_op_done(&env, 0, BC2_OK, 0, NULL, 0);          /* CCCD write response */
    fake_ev_notify(&env, 0, 0x10, n1, 3);                  /* attached-IO event right behind it */
    fake_ev_notify(&env, 0, 0x10, n2, 3);
    fake_tick(&env, 20);
    ASSERT_ACK(&env, 0, BC2_MSG_SUBSCRIBE, BC2_OK);
    ASSERT_TYPE(&env, 1, BC2_MSG_EVT_NOTIFY);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(0x10, fr.payload[1]);
    ASSERT_EQ(1, fr.payload[4]);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(2, fr.payload[4]);
    send_subscribe(&env, 0, 0x11, BC2_SUB_OFF);            /* unsubscribe before disconnect */
    ASSERT_EQ(BC2_SUB_OFF, fake_last_gcall(&env, GCALL_SUBSCRIBE)->mode);
}

/** Notifications of links that are not connected are ignored. */
static void test_notify_for_unknown_link(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    const uint8_t n[1] = {1};
    fake_ev_notify(&env, 5, 0x10, n, 1);
    fake_tick(&env, 5);
    ASSERT_EQ(0, fake_out_count(&env));
}

/** A refused droppable frame is counted, reported once per second and in GATT_STATUS_RSP. */
static void test_drop_counters(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t n[1] = {1};
    env.refuse_droppable = true;
    for (int i = 0; i < 3; i++) {
        fake_ev_notify(&env, 0, 0x10, n, 1);
    }
    env.gstats.dropped_notify = 4;                         /* drops inside the adapter */
    env.gstats.dropped_scan = 9;
    fake_tick(&env, 500);
    ASSERT_EQ(0, fake_out_count(&env));                    /* the report waits for the 1 s spacing */
    env.refuse_droppable = false;
    fake_tick(&env, 1000);
    ASSERT_EQ(2, fake_out_count(&env));
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_DROPPED, fr.type);
    ASSERT_EQ(BC2_DROP_SCAN, fr.payload[0]);
    ASSERT_EQ(9, fr.payload[1]);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_DROP_NOTIFY, fr.payload[0]);
    ASSERT_EQ(7, fr.payload[1]);                           /* 3 refused by the TX queue + 4 in the adapter */
    fake_tick(&env, 2000);
    ASSERT_EQ(2, fake_out_count(&env));                    /* nothing new: no event */
    fake_send(&env, BC2_MSG_GET_GATT_STATUS, NULL, 0);
    ASSERT_TRUE(fake_out_frame(&env, 2, &fr));
    ASSERT_EQ(BC2_MSG_GATT_STATUS_RSP, fr.type);
    ASSERT_EQ(17, fr.len);
    ASSERT_EQ(9, fr.payload[5]);
    ASSERT_EQ(7, fr.payload[7]);
}

/** GATT_STATUS_RSP carries the heap figures of the adapter. */
static void test_status_heap(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    env.gstats.free_heap = 0x00028000;
    env.gstats.min_free_heap = 0x00014000;
    fake_send(&env, BC2_MSG_GET_GATT_STATUS, NULL, 0);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(0x00028000u, (uint32_t)(fr.payload[9] | (fr.payload[10] << 8) | (fr.payload[11] << 16) |
                                      ((uint32_t)fr.payload[12] << 24)));
    ASSERT_EQ(0x14000, fr.payload[13] | (fr.payload[14] << 8) | (fr.payload[15] << 16));
}

/* ---- failsafe ---- */

static void test_failsafe_request(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    const uint8_t d[4] = {1, 2, 3, 4};
    send_failsafe(&env, 0, 0, 0x10, d, 4);
    ASSERT_ACK(&env, 0, BC2_MSG_SET_FAILSAFE, BC2_ERR_NOT_CONNECTED);
    make_connected(&env, 0, 64);
    send_failsafe(&env, 0, 4, 0x10, d, 4);                 /* slot out of range */
    ASSERT_ACK(&env, 0, BC2_MSG_SET_FAILSAFE, BC2_ERR_BAD_PARAM);
    send_failsafe(&env, 0, 3, 0x10, d, 4);
    ASSERT_ACK(&env, 1, BC2_MSG_SET_FAILSAFE, BC2_OK);
    uint8_t big[21] = {0};
    send_failsafe(&env, 0, 0, 0x10, big, 21);
    ASSERT_ACK(&env, 2, BC2_MSG_SET_FAILSAFE, BC2_ERR_PAYLOAD_TOO_LARGE);
}

/** WeDo 2.0 needs two stop writes: both run in slot order before the terminate, cleared slots do not. */
static void test_failsafe_order_on_keepalive_timeout(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    make_connected(&env, 0, 64);
    const uint8_t port1[4] = {1, 1, 1, 0}, port2[4] = {2, 1, 1, 0}, junk[2] = {9, 9};
    send_failsafe(&env, 0, 2, 0x20, port2, 4);             /* stored out of order on purpose */
    send_failsafe(&env, 0, 0, 0x20, port1, 4);
    send_failsafe(&env, 0, 1, 0x30, junk, 2);
    send_failsafe(&env, 0, 1, 0, NULL, 0);                 /* cleared again */
    fake_clear(&env);
    fake_tick(&env, 999 + 1000);                           /* keep-alive of 1000 ms expired (last frame at 0) */
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));
    ASSERT_EQ(1, env.gcalls[0].data[0]);                   /* slot 0 first */
    ASSERT_EQ(2, env.gcalls[1].data[0]);                   /* then slot 2 */
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 1999 + BC2_GATT_FAILSAFE_FLUSH_MS - 1);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 1999 + BC2_GATT_FAILSAFE_FLUSH_MS);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    ASSERT_EQ(env.gcalls[env.gcall_count - 1].kind, GCALL_TERMINATE);   /* the terminate is the last call */
}

/** A normal DISCONNECT does not run the failsafe (the host does its own cleanup). */
static void test_no_failsafe_on_disconnect(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t d[4] = {1, 2, 3, 4};
    send_failsafe(&env, 0, 0, 0x10, d, 4);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 0);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
}

/** Failsafe settings die with the link: a reconnect starts without them. */
static void test_failsafe_cleared_with_link(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t d[4] = {1, 2, 3, 4};
    send_failsafe(&env, 0, 0, 0x10, d, 4);
    fake_ev_link_down(&env, 0, 0x08);
    fake_tick(&env, 1);
    make_connected(&env, 0, 64);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_HELLO, (const uint8_t[]){1, 1, 0x10, 0x27}, 4);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
}

/** A controller that refuses the failsafe write (BUSY) is retried; after 200 ms the link is dropped anyway. */
static void test_failsafe_busy_retry(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t d[2] = {1, 2};
    send_failsafe(&env, 0, 0, 0x10, d, 2);
    env.write_cmd_busy = 2;
    send_conn_id_req(&env, BC2_MSG_PING, 0);
    fake_tick(&env, 20000 - 1);                            /* PING counts as a frame: no timeout, use the button */
    bc2_core_on_button(&env.core, 100);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));   /* busy once, nothing written yet */
    fake_tick(&env, 110);                                  /* busy twice */
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));
    fake_tick(&env, 120);                                  /* accepted */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_WRITE_CMD, 0));
    fake_tick(&env, 120 + BC2_GATT_FAILSAFE_FLUSH_MS);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
}

static void test_failsafe_busy_gives_up(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t d[2] = {1, 2};
    send_failsafe(&env, 0, 0, 0x10, d, 2);
    env.write_cmd_busy = 1000;                             /* controller never has room */
    bc2_core_on_button(&env.core, 100);
    fake_tick(&env, 299);
    ASSERT_EQ(0, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 300);                                  /* 200 ms limit: the link goes regardless */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
}

/* ---- safety stop ---- */

/** Keep-alive timeout: waiting and running connects end, links are terminated, the scan stops, then EVT_TIMEOUT. */
static void test_keepalive_timeout_stops_everything(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    make_connected(&env, 0, 64);
    send_connect(&env, 1, 0x11, BC2_CONNECT_FLAG_BACKGROUND, 0);   /* running attempt */
    send_connect(&env, 2, 0x12, 0, 9000);                          /* waiting */
    const uint8_t scan[7] = {1, 0, 0, 0, 0, 0, 0};
    fake_send(&env, BC2_MSG_SCAN_START, scan, sizeof(scan));
    const uint8_t sp[8] = {0, 1, 32, 0, 127, 0xF0, 0xFF, 0};
    fake_send(&env, BC2_MSG_SET_START, sp, sizeof(sp));            /* advertising set 0 */
    fake_clear(&env);

    fake_tick(&env, 1000);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_SCAN_STOP, 0xFF));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
    ASSERT_EQ(1, fake_count_calls(&env, CALL_STOP));               /* advertising set stopped as before */
    /* events: scan stopped, waiting connect ended, EVT_TIMEOUT last (all within this tick) */
    bc2_frame_t fr;
    int n = fake_out_count(&env);
    ASSERT_TRUE(n >= 3);
    ASSERT_TRUE(fake_out_frame(&env, n - 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_TIMEOUT, fr.type);
    bool saw_scan_stopped = false, saw_waiting_ended = false;
    for (int i = 0; i < n - 1; i++) {
        ASSERT_TRUE(fake_out_frame(&env, i, &fr));
        if (fr.type == BC2_MSG_EVT_SCAN_STOPPED) {
            saw_scan_stopped = fr.payload[0] == BC2_SCAN_STOP_SAFETY;
        }
        if (fr.type == BC2_MSG_EVT_DISCONNECTED && fr.payload[0] == 2) {
            saw_waiting_ended = fr.payload[2] == BC2_REASON_SAFETY;
        }
    }
    ASSERT_TRUE(saw_scan_stopped);
    ASSERT_TRUE(saw_waiting_ended);
    /* the adapter confirms: link 0 down, attempt of link 1 cancelled */
    fake_clear(&env);
    fake_ev_link_down(&env, 0, 0x16);
    fake_ev_connect_failed(&env, 1, 0x02);
    fake_tick(&env, 1010);
    ASSERT_EQ(2, fake_out_count(&env));
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_REASON_SAFETY, fr.payload[2]);           /* reported as stick safety, not the HCI code */
    ASSERT_TRUE(!bc2_core_gatt_active(&env.core));
    fake_tick(&env, 5000);                                 /* everything is gone: no reset */
    ASSERT_EQ(0, env.hard_resets);
}

/** Button: links, attempts and the scan stop, the session stays, new connections work afterwards. */
static void test_button_stops_links_keeps_session(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    const uint8_t scan[7] = {1, 0, 0, 0, 0, 0, 0};
    fake_send(&env, BC2_MSG_SCAN_START, scan, sizeof(scan));
    fake_clear(&env);
    bc2_core_on_button(&env.core, 50);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_SCAN_STOP, 0xFF));
    ASSERT_TRUE(fake_out_frame(&env, fake_out_count(&env) - 1, &fr));
    ASSERT_EQ(BC2_MSG_EVT_BUTTON, fr.type);
    fake_ev_link_down(&env, 0, 0x16);
    fake_tick(&env, 60);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_PING, NULL, 0);                /* session still alive */
    ASSERT_TYPE(&env, 0, BC2_MSG_PONG);
    send_connect(&env, 0, 0x40, 0, 0);                     /* the id can be used again */
    ASSERT_ACK(&env, 1, BC2_MSG_CONNECT, BC2_OK);
}

/** An accepted HELLO restarts the session: old links go, their ids stay busy until the link is down. */
static void test_hello_drops_links(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    make_connected(&env, 1, 64);
    const uint8_t scan[7] = {1, 0, 0, 0, 0, 0, 0};
    fake_send(&env, BC2_MSG_SCAN_START, scan, sizeof(scan));
    fake_clear(&env);
    fake_hello(&env, 1000);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 1));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_SCAN_STOP, 0xFF));
    send_connect(&env, 0, 0x77, 0, 0);                     /* old link still being torn down */
    bc2_frame_t fr;
    int n = fake_out_count(&env);
    ASSERT_TRUE(fake_out_frame(&env, n - 1, &fr));
    ASSERT_EQ(BC2_ERR_ALREADY_RUNNING, fr.payload[1]);
    fake_ev_link_down(&env, 0, 0x16);
    fake_ev_link_down(&env, 1, 0x16);
    fake_tick(&env, 10);
    send_connect(&env, 0, 0x77, 0, 0);
    n = fake_out_count(&env);
    ASSERT_TRUE(fake_out_frame(&env, n - 1, &fr));
    ASSERT_EQ(BC2_OK, fr.payload[1]);
}

/** A safety stop during a connect attempt also terminates the slot, so a link in the MTU wait cannot survive. */
static void test_safety_stop_terminates_attempt_slot(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    send_connect(&env, 1, 0x11, BC2_CONNECT_FLAG_BACKGROUND, 0);   /* running attempt (adapter may be in MTU wait) */
    fake_clear(&env);
    fake_tick(&env, 1000);                                 /* keep-alive timeout */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_CANCEL, 0xFF));
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 1));
    fake_tick(&env, 2000);                                 /* verify stage 1: both requests are repeated */
    ASSERT_TRUE(fake_count_gcalls(&env, GCALL_TERMINATE, 1) >= 2);
}

/** A link that survives the terminate request: terminated again after 1 s, the chip is reset after 2 s. */
static void test_surviving_link_hard_reset(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    make_connected(&env, 0, 64);
    fake_tick(&env, 1000);                                 /* keep-alive timeout at t = 1000 */
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 1999);
    ASSERT_EQ(0, env.hard_resets);
    const int before_retry = fake_count_gcalls(&env, GCALL_TERMINATE, 0);
    fake_tick(&env, 2000);                                 /* 1 s after the stop: the link is still there */
    ASSERT_TRUE(fake_count_gcalls(&env, GCALL_TERMINATE, 0) > before_retry);
    ASSERT_EQ(0, env.hard_resets);
    fake_tick(&env, 2999);
    ASSERT_EQ(0, env.hard_resets);
    fake_tick(&env, 3000);                                 /* 2 s after the stop: reset */
    ASSERT_EQ(1, env.hard_resets);
}

/** The terminate request is repeated every 500 ms while the link stays up. */
static void test_terminate_repeats(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    send_conn_id_req(&env, BC2_MSG_DISCONNECT, 0);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 499);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_tick(&env, 500);
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
    fake_ev_link_down(&env, 0, 0x16);
    fake_tick(&env, 600);
    fake_tick(&env, 1200);
    ASSERT_EQ(2, fake_count_gcalls(&env, GCALL_TERMINATE, 0));   /* link gone: no more requests */
    ASSERT_EQ(0, env.hard_resets);                               /* host disconnect never resets the chip */
}

/** A lost state event in the adapter means the link table is unreliable: drop everything. */
static void test_state_lost_safety_stop(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    make_connected(&env, 0, 64);
    env.gstats.state_lost = true;
    fake_tick(&env, 5);
    ASSERT_EQ(1, fake_count_gcalls(&env, GCALL_TERMINATE, 0));
}

/** HELLO of a 1.1 host (minor 1) and of a 1.0 host (minor 0) are both accepted. */
static void test_hello_minor_versions(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    const uint8_t v11[4] = {1, 1, 0xE8, 0x03}, v10[4] = {1, 0, 0xE8, 0x03};
    fake_send(&env, BC2_MSG_HELLO, v11, sizeof(v11));
    fake_send(&env, BC2_MSG_HELLO, v10, sizeof(v10));
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_HELLO_RSP, fr.type);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_HELLO_RSP, fr.type);
}

/** The session state of GATT does not leak into the next HELLO (counters restart). */
static void test_hello_resets_counters(void)
{
    fake_env_t env;
    fake_init(&env);
    open_session(&env);
    env.gstats.dropped_scan = 5;
    fake_tick(&env, 10);
    fake_hello(&env, 10000);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_GET_GATT_STATUS, NULL, 0);
    bc2_frame_t fr;
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(0, fr.payload[5]);
}

void run_gatt_core_tests(void)
{
    RUN_TEST(test_hello_capabilities);
    RUN_TEST(test_no_gatt_build);
    RUN_TEST(test_gatt_needs_session);
    RUN_TEST(test_gatt_bad_length);
    RUN_TEST(test_scan_start_stop_and_reports);
    RUN_TEST(test_scan_errors);
    RUN_TEST(test_scan_stopped_by_adapter);
    RUN_TEST(test_connect_flow);
    RUN_TEST(test_connect_with_default_mtu);
    RUN_TEST(test_connect_limit);
    RUN_TEST(test_connect_errors);
    RUN_TEST(test_connect_adapter_error);
    RUN_TEST(test_connect_serialised);
    RUN_TEST(test_connect_background_slice);
    RUN_TEST(test_connect_no_slice_when_alone);
    RUN_TEST(test_connect_timeout);
    RUN_TEST(test_connect_retry_until_deadline);
    RUN_TEST(test_connect_cancel_guard);
    RUN_TEST(test_disconnect_variants);
    RUN_TEST(test_orphan_and_raced_links);
    RUN_TEST(test_link_down_during_attempt);
    RUN_TEST(test_discovery);
    RUN_TEST(test_discovery_errors);
    RUN_TEST(test_discovery_timeout);
    RUN_TEST(test_read);
    RUN_TEST(test_procedure_errors);
    RUN_TEST(test_write_deferred_ack);
    RUN_TEST(test_write_cmd_busy);
    RUN_TEST(test_write_sizes);
    RUN_TEST(test_procedure_timeout);
    RUN_TEST(test_pending_on_link_loss);
    RUN_TEST(test_subscribe_ack_before_notify);
    RUN_TEST(test_notify_for_unknown_link);
    RUN_TEST(test_drop_counters);
    RUN_TEST(test_status_heap);
    RUN_TEST(test_failsafe_request);
    RUN_TEST(test_failsafe_order_on_keepalive_timeout);
    RUN_TEST(test_no_failsafe_on_disconnect);
    RUN_TEST(test_failsafe_cleared_with_link);
    RUN_TEST(test_failsafe_busy_retry);
    RUN_TEST(test_failsafe_busy_gives_up);
    RUN_TEST(test_keepalive_timeout_stops_everything);
    RUN_TEST(test_button_stops_links_keeps_session);
    RUN_TEST(test_hello_drops_links);
    RUN_TEST(test_surviving_link_hard_reset);
    RUN_TEST(test_terminate_repeats);
    RUN_TEST(test_safety_stop_terminates_attempt_slot);
    RUN_TEST(test_state_lost_safety_stop);
    RUN_TEST(test_hello_resets_counters);
    RUN_TEST(test_hello_minor_versions);
}
