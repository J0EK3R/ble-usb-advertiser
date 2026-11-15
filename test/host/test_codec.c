/**
 * @file test_codec.c
 * @brief Tests for request decoding (lengths, parameter validation) and response encoding
 *        (byte-exact golden vectors).
 */
#include "bc2_protocol.h"
#include "test_minunit.h"
#include "test_vectors.h"

/** Parse a golden vector into a frame (asserts that it is valid). */
static int parse_vector(const uint8_t *wire, size_t len, bc2_frame_t *out)
{
    bc2_framer_t f;
    bc2_framer_init(&f);
    for (size_t i = 0; i < len; i++) {
        bc2_framer_push(&f, wire[i], 0);
    }
    return bc2_framer_next(&f, out) ? 1 : 0;
}

/** Serialize @p fr and compare with a golden vector. Returns 1 when equal. */
static int matches_vector(const bc2_frame_t *fr, const uint8_t *wire, size_t len)
{
    uint8_t out[BC2_MAX_FRAME];
    return bc2_frame_encode(fr, out, sizeof(out)) == len && memcmp(out, wire, len) == 0;
}

static void test_decode_hello(void)
{
    bc2_frame_t fr;
    bc2_req_hello_t req;
    ASSERT_TRUE(parse_vector(GV_HELLO, sizeof(GV_HELLO), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_hello(&fr, &req));
    ASSERT_EQ(1, req.proto_major);
    ASSERT_EQ(0, req.proto_minor);
    ASSERT_EQ(1000, req.keepalive_ms);
    fr.len = 3;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_hello(&fr, &req));
    fr.len = 5;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_hello(&fr, &req));
}

static void test_decode_set_start(void)
{
    bc2_frame_t fr;
    bc2_req_set_start_t req;
    ASSERT_TRUE(parse_vector(GV_SET_START, sizeof(GV_SET_START), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_set_start(&fr, &req));
    ASSERT_EQ(0, req.set_id);
    ASSERT_EQ(1, req.flags);
    ASSERT_EQ(32, req.interval);
    ASSERT_EQ(127, req.tx_power);
    ASSERT_EQ(0xFFF0, req.mfr_id);
    ASSERT_EQ(8, req.data_len);
    ASSERT_EQ(1, req.data[0]);
    ASSERT_EQ(8, req.data[7]);
}

static void test_decode_set_start_lengths(void)
{
    bc2_frame_t fr;
    bc2_req_set_start_t req;
    ASSERT_TRUE(parse_vector(GV_SET_START, sizeof(GV_SET_START), &fr));
    fr.len--;                                                      /* one byte too short */
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_start(&fr, &req));
    fr.len += 2;                                                   /* one byte too long */
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_start(&fr, &req));
    fr.len = 7;                                                    /* shorter than the fixed part */
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_start(&fr, &req));
}

static void test_decode_set_start_params(void)
{
    bc2_frame_t fr;
    bc2_req_set_start_t req;
    ASSERT_TRUE(parse_vector(GV_SET_START, sizeof(GV_SET_START), &fr));

    fr.payload[1] = 0x01 | 0x08;                                   /* reserved flag bit 3 */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_set_start(&fr, &req));
    fr.payload[1] = 0x01 | (3u << BC2_FLAG_PDU_SHIFT);             /* reserved PDU type 3 */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_set_start(&fr, &req));
    fr.payload[1] = 0x01 | (BC2_PDU_ADV_NONCONN_IND << BC2_FLAG_PDU_SHIFT);
    ASSERT_EQ(BC2_OK, bc2_decode_set_start(&fr, &req));
    fr.payload[1] = 0x01;

    fr.payload[2] = 31;                                            /* interval 31 < 32 */
    fr.payload[3] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_set_start(&fr, &req));
    fr.payload[2] = 0x01;                                          /* interval 16385 > 16384 */
    fr.payload[3] = 0x40;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_set_start(&fr, &req));
    fr.payload[2] = 0x00;                                          /* interval 16384 is allowed */
    ASSERT_EQ(BC2_OK, bc2_decode_set_start(&fr, &req));
}

/** Build a SET_START frame with the given flags and data_len (data zero filled). */
static void make_set_start(bc2_frame_t *fr, uint8_t flags, uint8_t data_len)
{
    memset(fr, 0, sizeof(*fr));
    fr->type = BC2_MSG_SET_START;
    fr->len = (uint8_t)(8u + data_len);
    fr->payload[1] = flags;
    fr->payload[2] = 32;
    fr->payload[7] = data_len;
}

static void test_decode_set_start_size_limits(void)
{
    bc2_frame_t fr;
    bc2_req_set_start_t req;
    make_set_start(&fr, 0x01, 24);
    ASSERT_EQ(BC2_OK, bc2_decode_set_start(&fr, &req));
    make_set_start(&fr, 0x01, 25);
    ASSERT_EQ(BC2_ERR_PAYLOAD_TOO_LARGE, bc2_decode_set_start(&fr, &req));
    make_set_start(&fr, 0x00, 27);
    ASSERT_EQ(BC2_OK, bc2_decode_set_start(&fr, &req));
    make_set_start(&fr, 0x00, 28);
    ASSERT_EQ(BC2_ERR_PAYLOAD_TOO_LARGE, bc2_decode_set_start(&fr, &req));
    make_set_start(&fr, 0x01, 0);                                  /* empty payload is allowed */
    ASSERT_EQ(BC2_OK, bc2_decode_set_start(&fr, &req));
    /* data_len field inconsistent with the frame length */
    make_set_start(&fr, 0x01, 5);
    fr.payload[7] = 6;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_start(&fr, &req));
}

static void test_decode_set_update(void)
{
    bc2_frame_t fr;
    bc2_req_set_update_t req;
    ASSERT_TRUE(parse_vector(GV_SET_UPDATE, sizeof(GV_SET_UPDATE), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_set_update(&fr, &req));
    ASSERT_EQ(0, req.set_id);
    ASSERT_EQ(0xFFF0, req.mfr_id);
    ASSERT_EQ(4, req.data_len);
    ASSERT_EQ(0xA1, req.data[0]);
    fr.len--;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_update(&fr, &req));
    fr.len += 2;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_update(&fr, &req));
    fr.len = 3;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_update(&fr, &req));
    fr.len = 4 + 28;                                               /* 28 data bytes */
    fr.payload[3] = 28;
    ASSERT_EQ(BC2_ERR_PAYLOAD_TOO_LARGE, bc2_decode_set_update(&fr, &req));
}

static void test_decode_simple(void)
{
    bc2_frame_t fr;
    uint8_t id = 0;
    ASSERT_TRUE(parse_vector(GV_SET_STOP, sizeof(GV_SET_STOP), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_set_id(&fr, &id));
    fr.len = 0;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_id(&fr, &id));
    fr.len = 2;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_set_id(&fr, &id));
    ASSERT_TRUE(parse_vector(GV_PING, sizeof(GV_PING), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_empty(&fr));
    fr.len = 1;
    ASSERT_EQ(BC2_ERR_BAD_LENGTH, bc2_decode_empty(&fr));
}

/** The decoder accepts exactly what the golden request frames contain (PING/STOP_ALL/GET_STATUS). */
static void test_decode_golden_misc(void)
{
    bc2_frame_t fr;
    uint8_t id = 0;
    ASSERT_TRUE(parse_vector(GV_STOP_ALL, sizeof(GV_STOP_ALL), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_empty(&fr));
    ASSERT_TRUE(parse_vector(GV_GET_STATUS, sizeof(GV_GET_STATUS), &fr));
    ASSERT_EQ(BC2_OK, bc2_decode_set_id(&fr, &id));
    ASSERT_EQ(BC2_SET_ID_SUMMARY, id);
}

static void test_encode_hello_rsp(void)
{
    const bc2_hello_rsp_t rsp = {.status = 0, .proto_major = 1, .proto_minor = 0, .fw_major = 0,
                                 .fw_minor = 1, .fw_patch = 0, .max_sets = 5,
                                 .max_data_with_flags = 24, .max_data_no_flags = 27,
                                 .min_interval = 32, .max_interval = 16384, .keepalive_ms = 1000,
                                 .device_id = {0x24, 0x0A, 0xBC, 0x11, 0x22, 0x33}};
    bc2_frame_t fr;
    bc2_encode_hello_rsp(&fr, 1, &rsp);
    ASSERT_EQ(BC2_HELLO_RSP_LEN, fr.len);
    ASSERT_TRUE(matches_vector(&fr, GV_HELLO_RSP, sizeof(GV_HELLO_RSP)));
}

static void test_encode_ack(void)
{
    bc2_frame_t fr;
    const uint8_t tx = 127;
    const uint8_t us[4] = {0x7B, 0x03, 0x00, 0x00};
    uint8_t big[BC2_MAX_PAYLOAD];
    ASSERT_TRUE(bc2_encode_ack(&fr, 3, BC2_MSG_SET_START, BC2_OK, &tx, 1));
    ASSERT_TRUE(matches_vector(&fr, GV_ACK_SET_START, sizeof(GV_ACK_SET_START)));
    ASSERT_TRUE(bc2_encode_ack(&fr, 4, BC2_MSG_SET_UPDATE, BC2_OK, us, 4));
    ASSERT_TRUE(matches_vector(&fr, GV_ACK_SET_UPDATE, sizeof(GV_ACK_SET_UPDATE)));
    ASSERT_TRUE(!bc2_encode_ack(&fr, 0, 0, 0, big, sizeof(big)));   /* extra too large */
}

static void test_encode_pong_and_events(void)
{
    const bc2_pong_t pong = {.state = 1, .active_mask = 1, .parse_errors = 0, .uptime_ms = 10000};
    bc2_frame_t fr;
    bc2_encode_pong(&fr, 2, &pong);
    ASSERT_TRUE(matches_vector(&fr, GV_PONG, sizeof(GV_PONG)));
    bc2_encode_evt_timeout(&fr, 1000);
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_TIMEOUT, sizeof(GV_EVT_TIMEOUT)));
    bc2_encode_evt_button(&fr);
    ASSERT_EQ(BC2_MSG_EVT_BUTTON, fr.type);
    ASSERT_EQ(0, fr.len);
    bc2_encode_evt_set_stopped(&fr, 2, BC2_STOP_BLE_ERROR);
    ASSERT_EQ(2, fr.payload[0]);
    ASSERT_EQ(1, fr.payload[1]);
    ASSERT_EQ(0, fr.seq);
}

static void test_encode_set_status(void)
{
    const uint8_t data[3] = {1, 2, 3};
    const bc2_set_status_t st = {.set_id = 2, .running = 1, .flags = 1, .interval = 32,
                                 .tx_power = -3, .mfr_id = 0xC200, .data_len = 3, .data = data};
    bc2_frame_t fr;
    ASSERT_TRUE(bc2_encode_set_status(&fr, 9, &st));
    ASSERT_EQ(12, fr.len);
    ASSERT_EQ(0xFD, fr.payload[5]);                                 /* tx power -3 as two's complement */
    ASSERT_EQ(0x00, fr.payload[6]);                                 /* mfr 0xC200 little endian */
    ASSERT_EQ(0xC2, fr.payload[7]);
    ASSERT_EQ(3, fr.payload[8]);
    ASSERT_EQ(3, fr.payload[11]);
}

static void test_max_data_len(void)
{
    ASSERT_EQ(24, bc2_max_data_len(0x01));
    ASSERT_EQ(27, bc2_max_data_len(0x00));
}

void run_codec_tests(void)
{
    RUN_TEST(test_decode_hello);
    RUN_TEST(test_decode_set_start);
    RUN_TEST(test_decode_set_start_lengths);
    RUN_TEST(test_decode_set_start_params);
    RUN_TEST(test_decode_set_start_size_limits);
    RUN_TEST(test_decode_set_update);
    RUN_TEST(test_decode_simple);
    RUN_TEST(test_decode_golden_misc);
    RUN_TEST(test_encode_hello_rsp);
    RUN_TEST(test_encode_ack);
    RUN_TEST(test_encode_pong_and_events);
    RUN_TEST(test_encode_set_status);
    RUN_TEST(test_max_data_len);
}
