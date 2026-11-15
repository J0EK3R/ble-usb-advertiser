/**
 * @file test_core.c
 * @brief Tests for the bc2_core session state machine using a fake BLE adapter and fake clock.
 */
#include "bc2_core.h"
#include "fake_ble.h"
#include "test_minunit.h"

static const uint8_t DATA8[8] = {1, 2, 3, 4, 5, 6, 7, 8};

/** Assert that output frame @p idx is an ACK for @p orig with @p status. */
#define ASSERT_ACK(env, idx, orig, status)                                       \
    do {                                                                         \
        bc2_frame_t f_;                                                          \
        ASSERT_TRUE(fake_out_frame((env), (idx), &f_));                          \
        ASSERT_EQ(BC2_MSG_ACK, f_.type);                                         \
        ASSERT_EQ((orig), f_.payload[0]);                                        \
        ASSERT_EQ((status), f_.payload[1]);                                      \
    } while (0)

/** Requests before HELLO are refused with NO_SESSION, PING included. */
static void test_no_session(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_send(&env, BC2_MSG_PING, NULL, 0);
    ASSERT_ACK(&env, 0, BC2_MSG_SET_START, BC2_ERR_NO_SESSION);
    ASSERT_ACK(&env, 1, BC2_MSG_PING, BC2_ERR_NO_SESSION);
    ASSERT_EQ(0, env.call_count);
}

/** Unknown types, including the reserved GATT ranges, give UNKNOWN_CMD (even before HELLO). */
static void test_unknown_and_reserved_types(void)
{
    fake_env_t env;
    const uint8_t types[] = {0x00, 0x03, 0x15, 0x20, BC2_RESERVED_GATT_REQ_FIRST, 0x40,
                             BC2_RESERVED_GATT_REQ_LAST, BC2_RESERVED_GATT_RSP_FIRST,
                             BC2_RESERVED_GATT_EVT_LAST, BC2_MSG_ACK, 0xFF};
    fake_init(&env);
    for (size_t i = 0; i < sizeof(types); i++) {
        fake_clear(&env);
        fake_send(&env, types[i], NULL, 0);
        ASSERT_ACK(&env, 0, types[i], BC2_ERR_UNKNOWN_CMD);
    }
}

/** HELLO answers with capabilities. */
static void test_hello_ok(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    fake_hello(&env, 1000);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_HELLO_RSP, fr.type);
    ASSERT_EQ(BC2_HELLO_RSP_LEN, fr.len);
    ASSERT_MEM_EQ("BC2ADV", fr.payload, 6);
    ASSERT_EQ(5, fr.payload[12]);            /* max_sets */
    ASSERT_EQ(24, fr.payload[13]);
    ASSERT_EQ(27, fr.payload[14]);
    ASSERT_EQ(1000, fr.payload[19] | (fr.payload[20] << 8));
    ASSERT_EQ(BC2_LED_CONNECTED, bc2_core_led_state(&env.core, 0));
}

/** The set count reported follows the BLE adapter. */
static void test_hello_reports_adapter_max_sets(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    env.max_sets = 3;
    fake_hello(&env, 1000);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(3, fr.payload[12]);
}

/** A different proto major is refused and does not open a session. */
static void test_hello_wrong_major(void)
{
    fake_env_t env;
    const uint8_t p[4] = {2, 0, 0xE8, 0x03};
    fake_init(&env);
    fake_send(&env, BC2_MSG_HELLO, p, 4);
    ASSERT_ACK(&env, 0, BC2_MSG_HELLO, BC2_ERR_VERSION_UNSUPPORTED);
    fake_send(&env, BC2_MSG_PING, NULL, 0);
    ASSERT_ACK(&env, 1, BC2_MSG_PING, BC2_ERR_NO_SESSION);
}

/** Keep-alive clamping: 0 -> 1000, 50 -> 200, 99999 clamps to 10000. */
static void test_hello_keepalive_clamp(void)
{
    const uint16_t req[] = {0, 50, 200, 5000, 65535};
    const uint16_t want[] = {1000, 200, 200, 5000, 10000};
    for (size_t i = 0; i < sizeof(req) / sizeof(req[0]); i++) {
        fake_env_t env;
        bc2_frame_t fr;
        fake_init(&env);
        fake_hello(&env, req[i]);
        ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
        ASSERT_EQ(want[i], fr.payload[19] | (fr.payload[20] << 8));
    }
}

/** PING answers PONG; GET_STATUS(0xFF) too. */
static void test_ping_and_summary(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_clear(&env);
    env.now_ms = 123;
    fake_send(&env, BC2_MSG_PING, NULL, 0);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_PONG, fr.type);
    ASSERT_EQ(0, fr.payload[0]);
    ASSERT_EQ(123, fr.payload[5]);
    const uint8_t all = 0xFF;
    fake_send(&env, BC2_MSG_GET_STATUS, &all, 1);
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_PONG, fr.type);
}

/** SET_START success: BLE called once with the parameters, ACK carries applied tx power. */
static void test_set_start_ok(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_clear(&env);
    fake_start(&env, 2, 0xC200, DATA8, 8);
    ASSERT_EQ(1, fake_count_calls(&env, CALL_START));
    ASSERT_EQ(2, env.calls[0].set_id);
    ASSERT_EQ(0xC200, env.calls[0].mfr_id);
    ASSERT_MEM_EQ(DATA8, env.calls[0].data, 8);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_OK, fr.payload[1]);
    ASSERT_EQ(9, fr.payload[2]);                              /* applied tx from the fake */
    ASSERT_EQ(BC2_LED_ADVERTISING, bc2_core_led_state(&env.core, 0));
}

/** SET_START error cases: duplicate, out of range id, too large. */
static void test_set_start_errors(void)
{
    fake_env_t env;
    uint8_t big[25] = {0};
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_clear(&env);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);                     /* duplicate */
    fake_start(&env, 5, 0xFFF0, DATA8, 8);                     /* id == max_sets */
    fake_start(&env, 1, 0xFFF0, big, 25);                      /* 25 > 24 with flags */
    ASSERT_ACK(&env, 0, BC2_MSG_SET_START, BC2_OK);
    ASSERT_ACK(&env, 1, BC2_MSG_SET_START, BC2_ERR_ALREADY_RUNNING);
    ASSERT_ACK(&env, 2, BC2_MSG_SET_START, BC2_ERR_BAD_PARAM);
    ASSERT_ACK(&env, 3, BC2_MSG_SET_START, BC2_ERR_PAYLOAD_TOO_LARGE);
    ASSERT_EQ(1, fake_count_calls(&env, CALL_START));
}

/** A BLE error is reported with rc, the slot stays free and the LED shows ERROR. */
static void test_set_start_ble_error(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_clear(&env);
    env.start_result = BC2_ERR_BLE_ERROR;
    env.error_rc = 0x0203;
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_ERR_BLE_ERROR, fr.payload[1]);
    ASSERT_EQ(0x0203, fr.payload[2] | (fr.payload[3] << 8));
    ASSERT_EQ(BC2_LED_ERROR, bc2_core_led_state(&env.core, 0));
    ASSERT_EQ(BC2_LED_CONNECTED, bc2_core_led_state(&env.core, BC2_ERROR_DISPLAY_MS));
    env.start_result = BC2_OK;
    fake_start(&env, 0, 0xFFF0, DATA8, 8);                     /* slot was not marked running */
    ASSERT_ACK(&env, 1, BC2_MSG_SET_START, BC2_OK);
}

/** SET_UPDATE: ok (with processing time), not running, BLE error, size limit by flags. */
static void test_set_update(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    uint8_t p[4 + 25] = {0, 0xF0, 0xFF, 4, 9, 9, 9, 9};
    fake_init(&env);
    env.us_per_call = 250;                                      /* now_us advances by 250 us per call */
    fake_hello(&env, 1000);
    fake_send(&env, BC2_MSG_SET_UPDATE, p, 8);                  /* not running yet */
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_SET_UPDATE, p, 8);
    ASSERT_EQ(1, fake_count_calls(&env, CALL_UPDATE));
    ASSERT_MEM_EQ(p + 4, env.calls[0].data, 4);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_OK, fr.payload[1]);
    ASSERT_EQ(4, fr.len - 2);                                   /* extra = u32 processing time */
    ASSERT_EQ(250, fr.payload[2] | (fr.payload[3] << 8));      /* t0 and end differ by one call */
    ASSERT_TRUE(bc2_core_activity_flash(&env.core, 0));
    ASSERT_TRUE(!bc2_core_activity_flash(&env.core, BC2_ACTIVITY_FLASH_MS));

    p[3] = 25;                                                  /* 25 bytes > 24 (set uses AD flags) */
    fake_clear(&env);
    fake_send(&env, BC2_MSG_SET_UPDATE, p, 4 + 25);
    ASSERT_ACK(&env, 0, BC2_MSG_SET_UPDATE, BC2_ERR_PAYLOAD_TOO_LARGE);
    ASSERT_EQ(0, fake_count_calls(&env, CALL_UPDATE));

    p[3] = 4;
    env.update_result = BC2_ERR_BLE_ERROR;
    env.error_rc = 7;
    fake_send(&env, BC2_MSG_SET_UPDATE, p, 8);
    ASSERT_ACK(&env, 1, BC2_MSG_SET_UPDATE, BC2_ERR_BLE_ERROR);
    ASSERT_EQ(BC2_LED_ERROR, bc2_core_led_state(&env.core, 0));
}

/** Update of a set that is not running yields SET_NOT_FOUND. */
static void test_set_update_not_found(void)
{
    fake_env_t env;
    const uint8_t p[8] = {3, 0xF0, 0xFF, 4, 1, 2, 3, 4};
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_SET_UPDATE, p, 8);
    ASSERT_ACK(&env, 0, BC2_MSG_SET_UPDATE, BC2_ERR_SET_NOT_FOUND);
}

/** SET_STOP / STOP_ALL / GET_STATUS of a set. */
static void test_stop_and_status(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    const uint8_t id0 = 0, id1 = 1;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_start(&env, 1, 0xFF00, DATA8, 4);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_GET_STATUS, &id1, 1);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_SET_STATUS_RSP, fr.type);
    ASSERT_EQ(1, fr.payload[1]);                                /* running */
    ASSERT_EQ(4, fr.payload[8]);                                /* data_len */
    fake_send(&env, BC2_MSG_SET_STOP, &id0, 1);
    ASSERT_ACK(&env, 1, BC2_MSG_SET_STOP, BC2_OK);
    fake_send(&env, BC2_MSG_SET_STOP, &id0, 1);
    ASSERT_ACK(&env, 2, BC2_MSG_SET_STOP, BC2_ERR_SET_NOT_FOUND);
    fake_send(&env, BC2_MSG_GET_STATUS, &id0, 1);
    ASSERT_TRUE(fake_out_frame(&env, 3, &fr));
    ASSERT_EQ(0, fr.payload[1]);                                /* no longer running */
    fake_send(&env, BC2_MSG_STOP_ALL, NULL, 0);
    ASSERT_ACK(&env, 4, BC2_MSG_STOP_ALL, BC2_OK);
    ASSERT_EQ(2, fake_count_calls(&env, CALL_STOP));            /* set 0 once, set 1 once */
    ASSERT_EQ(BC2_LED_CONNECTED, bc2_core_led_state(&env.core, 0));
}

/** Wrong payload lengths give BAD_LENGTH. */
static void test_bad_length(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_clear(&env);
    fake_send(&env, BC2_MSG_PING, DATA8, 1);
    fake_send(&env, BC2_MSG_SET_STOP, NULL, 0);
    fake_send(&env, BC2_MSG_STOP_ALL, DATA8, 2);
    ASSERT_ACK(&env, 0, BC2_MSG_PING, BC2_ERR_BAD_LENGTH);
    ASSERT_ACK(&env, 1, BC2_MSG_SET_STOP, BC2_ERR_BAD_LENGTH);
    ASSERT_ACK(&env, 2, BC2_MSG_STOP_ALL, BC2_ERR_BAD_LENGTH);
}

/** A second HELLO stops all running sets and clears the table. */
static void test_hello_stops_sets(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_start(&env, 4, 0xFFF0, DATA8, 8);
    fake_clear(&env);
    fake_hello(&env, 1000);
    ASSERT_EQ(2, fake_count_calls(&env, CALL_STOP));
    fake_start(&env, 0, 0xFFF0, DATA8, 8);                      /* slot is free again */
    ASSERT_ACK(&env, 1, BC2_MSG_SET_START, BC2_OK);
}

/** Keep-alive timeout: nothing at keepalive-1, everything stopped at keepalive. */
static void test_keepalive_timeout(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    env.now_ms = 1000;
    fake_hello(&env, 500);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_start(&env, 1, 0xFFF0, DATA8, 8);
    fake_clear(&env);
    bc2_core_tick(&env.core, 1000 + 499);
    ASSERT_EQ(0, fake_count_calls(&env, CALL_STOP));
    ASSERT_EQ(0, env.out_len);
    bc2_core_tick(&env.core, 1000 + 500);
    ASSERT_EQ(2, fake_count_calls(&env, CALL_STOP));            /* ble.stop for every running set */
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_TIMEOUT, fr.type);
    ASSERT_EQ(500, fr.payload[0] | (fr.payload[1] << 8));
    ASSERT_EQ(BC2_LED_TIMEOUT, bc2_core_led_state(&env.core, 1600));
    env.now_ms = 1600;
    fake_send(&env, BC2_MSG_PING, NULL, 0);                      /* session is gone */
    ASSERT_ACK(&env, 1, BC2_MSG_PING, BC2_ERR_NO_SESSION);
    fake_hello(&env, 500);                                       /* HELLO re-opens, clears TIMEOUT */
    ASSERT_EQ(BC2_LED_CONNECTED, bc2_core_led_state(&env.core, 1600));
}

/** Any valid frame (PING) postpones the timeout; garbage does not. */
static void test_keepalive_postponed_by_valid_frames_only(void)
{
    fake_env_t env;
    const uint8_t garbage[] = {1, 2, 3, 0xB2, 0xB2, 0x05, 9, 9, 9};
    fake_init(&env);
    fake_hello(&env, 400);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    env.now_ms = 300;
    fake_send(&env, BC2_MSG_PING, NULL, 0);
    bc2_core_tick(&env.core, 600);                               /* 300 ms since the PING: alive */
    ASSERT_EQ(BC2_LED_ADVERTISING, bc2_core_led_state(&env.core, 600));
    bc2_core_on_bytes(&env.core, garbage, sizeof(garbage), 650);
    bc2_core_tick(&env.core, 700);                               /* 400 ms since the PING: timeout */
    ASSERT_EQ(BC2_LED_TIMEOUT, bc2_core_led_state(&env.core, 5000));
}

/** Button stops all sets, keeps the session, emits EVT_BUTTON and flashes the LED. */
static void test_button(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_clear(&env);
    bc2_core_on_button(&env.core, 100);
    ASSERT_EQ(1, fake_count_calls(&env, CALL_STOP));
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_BUTTON, fr.type);
    ASSERT_EQ(BC2_LED_BUTTON_STOP, bc2_core_led_state(&env.core, 200));
    ASSERT_EQ(BC2_LED_CONNECTED, bc2_core_led_state(&env.core, 100 + BC2_BUTTON_DISPLAY_MS));
    env.now_ms = 150;
    fake_send(&env, BC2_MSG_PING, NULL, 0);                      /* session still alive */
    ASSERT_TRUE(fake_out_frame(&env, 1, &fr));
    ASSERT_EQ(BC2_MSG_PONG, fr.type);
}

/** A stop that fails twice triggers the hard reset; a single failure is retried silently. */
static void test_stop_failure_hard_reset(void)
{
    fake_env_t env;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    fake_start(&env, 1, 0xFFF0, DATA8, 8);
    env.stop_failures = 1;
    bc2_core_on_button(&env.core, 0);
    ASSERT_EQ(0, env.hard_resets);                               /* retry succeeded */
    fake_start(&env, 0, 0xFFF0, DATA8, 8);
    env.stop_failures = 2;
    fake_send(&env, BC2_MSG_STOP_ALL, NULL, 0);
    ASSERT_EQ(1, env.hard_resets);
}

/** Asynchronous BLE failure: slot freed, EVT_SET_STOPPED sent, LED ERROR. */
static void test_async_failure(void)
{
    fake_env_t env;
    bc2_frame_t fr;
    fake_init(&env);
    fake_hello(&env, 1000);
    fake_start(&env, 3, 0xFFF0, DATA8, 8);
    fake_clear(&env);
    env.failure_pending = true;
    env.failure_set = 3;
    env.failure_reason = BC2_STOP_DISCONNECT_FAILED;
    bc2_core_tick(&env.core, 10);
    ASSERT_TRUE(fake_out_frame(&env, 0, &fr));
    ASSERT_EQ(BC2_MSG_EVT_SET_STOPPED, fr.type);
    ASSERT_EQ(3, fr.payload[0]);
    ASSERT_EQ(2, fr.payload[1]);
    ASSERT_EQ(BC2_LED_ERROR, bc2_core_led_state(&env.core, 10));
}

/** Parse errors turn the LED to ERROR for 2 s; BOOT and NO_HOST depend on BLE readiness. */
static void test_led_states(void)
{
    fake_env_t env;
    uint8_t bad[6] = {BC2_SOF, 0x02, 0x02, 0x01, 0x00, 0x00};   /* wrong CRC */
    fake_init(&env);
    env.ready = false;
    ASSERT_EQ(BC2_LED_BOOT, bc2_core_led_state(&env.core, 0));
    env.ready = true;
    ASSERT_EQ(BC2_LED_NO_HOST, bc2_core_led_state(&env.core, 0));
    bc2_core_on_bytes(&env.core, bad, sizeof(bad), 50);
    ASSERT_EQ(BC2_LED_ERROR, bc2_core_led_state(&env.core, 51));
    ASSERT_EQ(BC2_LED_NO_HOST, bc2_core_led_state(&env.core, 50 + BC2_ERROR_DISPLAY_MS));
}

void run_core_tests(void)
{
    RUN_TEST(test_no_session);
    RUN_TEST(test_unknown_and_reserved_types);
    RUN_TEST(test_hello_ok);
    RUN_TEST(test_hello_reports_adapter_max_sets);
    RUN_TEST(test_hello_wrong_major);
    RUN_TEST(test_hello_keepalive_clamp);
    RUN_TEST(test_ping_and_summary);
    RUN_TEST(test_set_start_ok);
    RUN_TEST(test_set_start_errors);
    RUN_TEST(test_set_start_ble_error);
    RUN_TEST(test_set_update);
    RUN_TEST(test_set_update_not_found);
    RUN_TEST(test_stop_and_status);
    RUN_TEST(test_bad_length);
    RUN_TEST(test_hello_stops_sets);
    RUN_TEST(test_keepalive_timeout);
    RUN_TEST(test_keepalive_postponed_by_valid_frames_only);
    RUN_TEST(test_button);
    RUN_TEST(test_stop_failure_hard_reset);
    RUN_TEST(test_async_failure);
    RUN_TEST(test_led_states);
}
