/**
 * @file test_gatt_codec.c
 * @brief Tests for the protocol 1.1 codecs: golden vectors (shared with the Python tests),
 *        BAD_LENGTH / BAD_PARAM / PAYLOAD_TOO_LARGE of every request and maximum-size messages.
 */
#include "bc2_gatt_protocol.h"
#include "test_gatt_vectors.h"
#include "test_minunit.h"
#include "test_vectors.h"

/** Parse a golden vector into a frame; returns 1 on success. */
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

/** Load golden vector @p v into frame @p fr, failing the test if it is not a valid frame. */
#define LOAD(fr, v) ASSERT_TRUE(parse_vector((v), sizeof(v), &(fr)))
/** Check that @p fr with its payload length changed to @p n gives @p expected from @p call. */
#define ASSERT_LEN_STATUS(fr, n, expected, call) \
    do {                                         \
        (fr).len = (uint8_t)(n);                 \
        ASSERT_EQ((expected), (call));           \
    } while (0)

static void test_is_gatt_request(void)
{
    ASSERT_TRUE(!bc2_is_gatt_request(0x2F));
    ASSERT_TRUE(bc2_is_gatt_request(0x30));
    ASSERT_TRUE(bc2_is_gatt_request(0x3A));
    ASSERT_TRUE(!bc2_is_gatt_request(0x3B));
    ASSERT_TRUE(!bc2_is_gatt_request(0xC0));
}

static void test_decode_scan_start(void)
{
    bc2_frame_t fr;
    bc2_req_scan_start_t r;
    LOAD(fr, GV_SCAN_START_DEFAULT);                       /* 0/0/0 -> defaults */
    ASSERT_EQ(BC2_OK, bc2_decode_scan_start(&fr, &r));
    ASSERT_EQ(BC2_SCAN_FLAG_ACTIVE, r.flags);
    ASSERT_EQ(160, r.interval);
    ASSERT_EQ(80, r.window);
    ASSERT_EQ(1000, r.refresh_ms);
    LOAD(fr, GV_SCAN_START_EXPLICIT);
    ASSERT_EQ(BC2_OK, bc2_decode_scan_start(&fr, &r));
    ASSERT_EQ(3, r.flags);
    ASSERT_EQ(160, r.interval);
    ASSERT_EQ(80, r.window);
    ASSERT_EQ(1000, r.refresh_ms);

    ASSERT_LEN_STATUS(fr, 6, BC2_ERR_BAD_LENGTH, bc2_decode_scan_start(&fr, &r));
    ASSERT_LEN_STATUS(fr, 8, BC2_ERR_BAD_LENGTH, bc2_decode_scan_start(&fr, &r));
    LOAD(fr, GV_SCAN_START_EXPLICIT);
    fr.payload[0] = 0x05;                                  /* reserved flag bit */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_scan_start(&fr, &r));
    LOAD(fr, GV_SCAN_START_EXPLICIT);
    fr.payload[3] = 0xA1;                                  /* window 161 > interval 160 */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_scan_start(&fr, &r));
    LOAD(fr, GV_SCAN_START_EXPLICIT);
    fr.payload[1] = 3;                                     /* interval 3 below minimum 4 */
    fr.payload[2] = 0;
    fr.payload[3] = 3;
    fr.payload[4] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_scan_start(&fr, &r));
    LOAD(fr, GV_SCAN_START_EXPLICIT);
    fr.payload[1] = 0;                                     /* interval 0 but window 80: invalid mix */
    fr.payload[2] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_scan_start(&fr, &r));
    LOAD(fr, GV_SCAN_START_EXPLICIT);
    fr.payload[5] = 0x11;                                  /* refresh 10001 ms > 10000 */
    fr.payload[6] = 0x27;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_scan_start(&fr, &r));
}

static void test_decode_connect(void)
{
    static const uint8_t addr1[6] = {0x90, 0x84, 0x2B, 0x01, 0x02, 0x03};
    bc2_frame_t fr;
    bc2_req_connect_t r;
    LOAD(fr, GV_CONNECT_BACKGROUND);
    ASSERT_EQ(BC2_OK, bc2_decode_connect(&fr, &r));
    ASSERT_EQ(0, r.conn_id);
    ASSERT_MEM_EQ(addr1, r.addr, 6);
    ASSERT_EQ(BC2_ADDR_TYPE_AUTO, r.addr_type);
    ASSERT_EQ(BC2_CONNECT_FLAG_BACKGROUND, r.flags);
    ASSERT_EQ(0, r.timeout_ms);                            /* background: no timeout */
    ASSERT_EQ(12, r.itvl_min);                             /* 0/0 -> 15..30 ms */
    ASSERT_EQ(24, r.itvl_max);
    LOAD(fr, GV_CONNECT_EXPLICIT);
    ASSERT_EQ(BC2_OK, bc2_decode_connect(&fr, &r));
    ASSERT_EQ(2, r.conn_id);
    ASSERT_EQ(1, r.addr_type);
    ASSERT_EQ(0xFF, r.addr[5]);
    ASSERT_EQ(5000, r.timeout_ms);
    ASSERT_EQ(12, r.itvl_min);
    ASSERT_EQ(24, r.itvl_max);

    LOAD(fr, GV_CONNECT_EXPLICIT);                         /* timeout 0 without background -> 10 s */
    fr.payload[9] = 0;
    fr.payload[10] = 0;
    ASSERT_EQ(BC2_OK, bc2_decode_connect(&fr, &r));
    ASSERT_EQ(10000, r.timeout_ms);

    ASSERT_LEN_STATUS(fr, 14, BC2_ERR_BAD_LENGTH, bc2_decode_connect(&fr, &r));
    ASSERT_LEN_STATUS(fr, 16, BC2_ERR_BAD_LENGTH, bc2_decode_connect(&fr, &r));
    LOAD(fr, GV_CONNECT_EXPLICIT);
    fr.payload[0] = 16;                                    /* conn_id above 15 */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_connect(&fr, &r));
    LOAD(fr, GV_CONNECT_EXPLICIT);
    fr.payload[7] = 2;                                     /* addr_type 2 is not allowed */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_connect(&fr, &r));
    LOAD(fr, GV_CONNECT_EXPLICIT);
    fr.payload[8] = 0x02;                                  /* reserved flag */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_connect(&fr, &r));
    LOAD(fr, GV_CONNECT_EXPLICIT);
    fr.payload[11] = 5;                                    /* itvl_min 5 < 6 */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_connect(&fr, &r));
    LOAD(fr, GV_CONNECT_EXPLICIT);
    fr.payload[13] = 11;                                   /* itvl_max 11 < itvl_min 12 */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_connect(&fr, &r));
    LOAD(fr, GV_CONNECT_EXPLICIT);
    fr.payload[13] = 0x81;                                 /* itvl_max 3201 > 3200 */
    fr.payload[14] = 0x0C;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_connect(&fr, &r));
}

static void test_decode_conn_id_requests(void)
{
    bc2_frame_t fr;
    uint8_t id = 0xEE;
    LOAD(fr, GV_DISCONNECT);
    ASSERT_EQ(BC2_OK, bc2_decode_conn_id(&fr, &id));
    ASSERT_EQ(1, id);
    LOAD(fr, GV_DISCOVER);
    ASSERT_EQ(BC2_OK, bc2_decode_conn_id(&fr, &id));
    ASSERT_LEN_STATUS(fr, 0, BC2_ERR_BAD_LENGTH, bc2_decode_conn_id(&fr, &id));
    ASSERT_LEN_STATUS(fr, 2, BC2_ERR_BAD_LENGTH, bc2_decode_conn_id(&fr, &id));
    fr.len = 1;
    fr.payload[0] = 16;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_conn_id(&fr, &id));
    fr.payload[0] = 15;                                    /* highest valid id */
    ASSERT_EQ(BC2_OK, bc2_decode_conn_id(&fr, &id));
    /* SCAN_STOP and GET_GATT_STATUS carry no payload and use bc2_decode_empty() */
    LOAD(fr, GV_SCAN_STOP);
    ASSERT_EQ(BC2_OK, bc2_decode_empty(&fr));
    LOAD(fr, GV_GET_GATT_STATUS);
    ASSERT_EQ(BC2_OK, bc2_decode_empty(&fr));
}

static void test_decode_read(void)
{
    bc2_frame_t fr;
    bc2_req_read_t r;
    LOAD(fr, GV_READ);
    ASSERT_EQ(BC2_OK, bc2_decode_read(&fr, &r));
    ASSERT_EQ(1, r.conn_id);
    ASSERT_EQ(0x12, r.handle);
    ASSERT_LEN_STATUS(fr, 2, BC2_ERR_BAD_LENGTH, bc2_decode_read(&fr, &r));
    ASSERT_LEN_STATUS(fr, 4, BC2_ERR_BAD_LENGTH, bc2_decode_read(&fr, &r));
    LOAD(fr, GV_READ);
    fr.payload[1] = 0;                                     /* handle 0 is invalid */
    fr.payload[2] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_read(&fr, &r));
}

static void test_decode_write(void)
{
    bc2_frame_t fr;
    bc2_req_write_t r;
    LOAD(fr, GV_WRITE_DUPLO);
    ASSERT_EQ(BC2_OK, bc2_decode_write(&fr, &r));
    ASSERT_EQ(0, r.conn_id);
    ASSERT_EQ(0x10, r.handle);
    ASSERT_EQ(8, r.data_len);
    ASSERT_EQ(0x81, r.data[2]);
    LOAD(fr, GV_WRITE_CMD_MKDIY);                          /* 18 byte MouldKing DIY frame */
    ASSERT_EQ(BC2_OK, bc2_decode_write(&fr, &r));
    ASSERT_EQ(18, r.data_len);
    ASSERT_EQ(0x33, r.data[17]);
    LOAD(fr, GV_WRITE_CMD_MAX);                            /* 56 data bytes = 59 byte payload */
    ASSERT_EQ(59, fr.len);
    ASSERT_EQ(BC2_OK, bc2_decode_write(&fr, &r));
    ASSERT_EQ(56, r.data_len);
    ASSERT_EQ(55, r.data[55]);

    fr.len = 60;                                           /* 57 data bytes */
    ASSERT_EQ(BC2_ERR_PAYLOAD_TOO_LARGE, bc2_decode_write(&fr, &r));
    ASSERT_LEN_STATUS(fr, 3, BC2_ERR_BAD_LENGTH, bc2_decode_write(&fr, &r));   /* no data byte */
    ASSERT_LEN_STATUS(fr, 0, BC2_ERR_BAD_LENGTH, bc2_decode_write(&fr, &r));
    fr.len = 4;
    fr.payload[1] = 0;
    fr.payload[2] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_write(&fr, &r));   /* handle 0 */
    fr.payload[2] = 1;
    fr.payload[0] = 16;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_write(&fr, &r));   /* conn_id 16 */
}

static void test_decode_subscribe(void)
{
    bc2_frame_t fr;
    bc2_req_subscribe_t r;
    LOAD(fr, GV_SUBSCRIBE);
    ASSERT_EQ(BC2_OK, bc2_decode_subscribe(&fr, &r));
    ASSERT_EQ(0, r.conn_id);
    ASSERT_EQ(0x11, r.cccd_handle);
    ASSERT_EQ(BC2_SUB_NOTIFY, r.mode);
    fr.payload[3] = BC2_SUB_INDICATE;
    ASSERT_EQ(BC2_OK, bc2_decode_subscribe(&fr, &r));
    fr.payload[3] = BC2_SUB_OFF;
    ASSERT_EQ(BC2_OK, bc2_decode_subscribe(&fr, &r));
    fr.payload[3] = 3;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_subscribe(&fr, &r));
    ASSERT_LEN_STATUS(fr, 3, BC2_ERR_BAD_LENGTH, bc2_decode_subscribe(&fr, &r));
    ASSERT_LEN_STATUS(fr, 5, BC2_ERR_BAD_LENGTH, bc2_decode_subscribe(&fr, &r));
    LOAD(fr, GV_SUBSCRIBE);
    fr.payload[1] = 0;                                     /* CCCD handle 0 */
    fr.payload[2] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_subscribe(&fr, &r));
}

static void test_decode_set_failsafe(void)
{
    bc2_frame_t fr;
    bc2_req_set_failsafe_t r;
    LOAD(fr, GV_SET_FAILSAFE_MKDIY);
    ASSERT_EQ(BC2_OK, bc2_decode_set_failsafe(&fr, &r));
    ASSERT_EQ(1, r.conn_id);
    ASSERT_EQ(0, r.slot);
    ASSERT_EQ(0x2A, r.handle);
    ASSERT_EQ(18, r.data_len);                             /* the 18 byte MouldKing DIY frame */
    LOAD(fr, GV_SET_FAILSAFE_CLEAR);
    ASSERT_EQ(BC2_OK, bc2_decode_set_failsafe(&fr, &r));
    ASSERT_EQ(1, r.slot);
    ASSERT_EQ(0, r.data_len);                              /* empty data clears the slot */
    fr.payload[1] = 3;                                     /* highest slot */
    ASSERT_EQ(BC2_OK, bc2_decode_set_failsafe(&fr, &r));
    fr.payload[1] = 4;                                     /* slot out of range */
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_set_failsafe(&fr, &r));
    fr.payload[1] = 0;
    ASSERT_LEN_STATUS(fr, 3, BC2_ERR_BAD_LENGTH, bc2_decode_set_failsafe(&fr, &r));
    fr.len = 4 + 21;                                       /* 21 data bytes */
    ASSERT_EQ(BC2_ERR_PAYLOAD_TOO_LARGE, bc2_decode_set_failsafe(&fr, &r));
    fr.len = 4 + 20;                                       /* exactly the limit */
    ASSERT_EQ(BC2_OK, bc2_decode_set_failsafe(&fr, &r));
    fr.payload[2] = 0;                                     /* handle 0 with data */
    fr.payload[3] = 0;
    ASSERT_EQ(BC2_ERR_BAD_PARAM, bc2_decode_set_failsafe(&fr, &r));
}

static void test_encode_responses(void)
{
    static const uint8_t rd[2] = {0xA1, 0xB2};
    bc2_frame_t fr;
    ASSERT_TRUE(bc2_encode_read_rsp(&fr, 15, 1, 0x12, 0, rd, 2));
    ASSERT_TRUE(matches_vector(&fr, GV_READ_RSP, sizeof(GV_READ_RSP)));
    const bc2_gatt_status_t st = {.scanning = 1, .connecting_mask = 4, .connected_mask = 3,
                                  .dropped_scan = 2, .dropped_notify = 0,
                                  .free_heap = 0x00028000, .min_free_heap = 0x00014000};
    bc2_encode_gatt_status_rsp(&fr, 20, &st);
    ASSERT_TRUE(matches_vector(&fr, GV_GATT_STATUS_RSP, sizeof(GV_GATT_STATUS_RSP)));
    const uint8_t att = 3;
    ASSERT_TRUE(bc2_encode_ack(&fr, 16, BC2_MSG_WRITE, BC2_ERR_ATT_ERROR, &att, 1));
    ASSERT_TRUE(matches_vector(&fr, GV_ACK_WRITE_ATT_ERROR, sizeof(GV_ACK_WRITE_ATT_ERROR)));
    ASSERT_TRUE(bc2_encode_ack(&fr, 11, BC2_MSG_CONNECT, BC2_OK, NULL, 0));
    ASSERT_TRUE(matches_vector(&fr, GV_ACK_CONNECT_OK, sizeof(GV_ACK_CONNECT_OK)));
}

static void test_encode_events(void)
{
    static const uint8_t notify[5] = {0x04, 0x00, 0x45, 0x00, 0x01};
    static const uint8_t addr[6] = {0x90, 0x84, 0x2B, 0x01, 0x02, 0x03};
    static const uint8_t adv[13] = {0x02, 0x01, 0x06, 0x09, 0xFF, 0x97, 0x03,
                                    0x00, 0x41, 0x20, 0x00, 0x00, 0x00};
    static const uint8_t uuid128[16] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x16,
                                        0xDE, 0xEF, 0x12, 0x12, 0x23, 0x16, 0x00, 0x00};
    static const uint8_t uuid_chr[16] = {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x16,
                                         0xDE, 0xEF, 0x12, 0x12, 0x24, 0x16, 0x00, 0x00};
    static const uint8_t uuid16[2] = {0x0A, 0x18};
    bc2_frame_t fr;
    ASSERT_TRUE(bc2_encode_evt_notify(&fr, 0, 0x11, 0, notify, sizeof(notify)));
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_NOTIFY, sizeof(GV_EVT_NOTIFY)));
    bc2_encode_evt_connected(&fr, 0, 64, 24, addr);
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_CONNECTED, sizeof(GV_EVT_CONNECTED)));
    bc2_encode_evt_disconnected(&fr, 0, BC2_PHASE_CONNECTED, 0x08);
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_DISCONNECTED, sizeof(GV_EVT_DISCONNECTED)));
    ASSERT_TRUE(bc2_encode_evt_scan_report(&fr, addr, 1, -60, BC2_REPORT_CONNECTABLE, adv, sizeof(adv)));
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_SCAN_REPORT, sizeof(GV_EVT_SCAN_REPORT)));
    ASSERT_TRUE(bc2_encode_evt_gatt_service(&fr, 0, 0x10, 0x20, uuid128, 16));
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_GATT_SERVICE_128, sizeof(GV_EVT_GATT_SERVICE_128)));
    ASSERT_TRUE(bc2_encode_evt_gatt_service(&fr, 0, 0x30, 0x3F, uuid16, 2));
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_GATT_SERVICE_16, sizeof(GV_EVT_GATT_SERVICE_16)));
    ASSERT_TRUE(bc2_encode_evt_gatt_char(&fr, 0, 0x11, 0x12, 0x1E, uuid_chr, 16));
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_GATT_CHAR, sizeof(GV_EVT_GATT_CHAR)));
    bc2_encode_evt_discover_done(&fr, 0, BC2_OK, 2, 5);
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_DISCOVER_DONE, sizeof(GV_EVT_DISCOVER_DONE)));
    bc2_encode_evt_scan_stopped(&fr, BC2_SCAN_STOP_ERROR, 15);
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_SCAN_STOPPED, sizeof(GV_EVT_SCAN_STOPPED)));
    bc2_encode_evt_dropped(&fr, BC2_DROP_NOTIFY, 42);
    ASSERT_TRUE(matches_vector(&fr, GV_EVT_DROPPED, sizeof(GV_EVT_DROPPED)));
}

/** Largest messages: values of 56 bytes, scan data of 31 bytes, rejects beyond that. */
static void test_encode_limits(void)
{
    uint8_t big[BC2_MAX_PAYLOAD] = {0};
    bc2_frame_t fr;
    ASSERT_TRUE(bc2_encode_evt_notify(&fr, 1, 2, BC2_VALUE_TRUNCATED, big, 56));
    ASSERT_EQ(60, fr.len);                                 /* exactly one full frame */
    ASSERT_TRUE(!bc2_encode_evt_notify(&fr, 1, 2, 0, big, 57));
    ASSERT_TRUE(bc2_encode_read_rsp(&fr, 1, 1, 2, BC2_READ_TRUNCATED, big, 56));
    ASSERT_EQ(60, fr.len);
    ASSERT_TRUE(!bc2_encode_read_rsp(&fr, 1, 1, 2, 0, big, 57));
    static const uint8_t addr[6] = {1, 2, 3, 4, 5, 6};
    ASSERT_TRUE(bc2_encode_evt_scan_report(&fr, addr, 0, -1, 0, big, 31));
    ASSERT_EQ(41, fr.len);
    ASSERT_TRUE(!bc2_encode_evt_scan_report(&fr, addr, 0, -1, 0, big, 32));
    ASSERT_TRUE(!bc2_encode_evt_gatt_service(&fr, 0, 1, 2, big, 4));   /* only 2 or 16 byte UUIDs */
    ASSERT_TRUE(!bc2_encode_evt_gatt_char(&fr, 0, 1, 0, 0, big, 0));
}

/** HELLO_RSP announces the capability block for any max_conns from 1 to 16. */
static void test_hello_rsp_max_conns_range(void)
{
    for (uint8_t n = 1; n <= BC2_GATT_MAX_CONNS_LIMIT; n++) {
        bc2_hello_rsp_t rsp = {0};
        rsp.max_conns = n;
        rsp.gatt_flags = BC2_GATT_FLAG_SCAN | BC2_GATT_FLAG_CENTRAL | BC2_GATT_FLAG_FAILSAFE;
        bc2_frame_t fr;
        bc2_encode_hello_rsp(&fr, 1, &rsp);
        ASSERT_EQ(32, fr.len);
        ASSERT_EQ(n, fr.payload[28]);
        ASSERT_EQ(7, fr.payload[27]);
    }
}

void run_gatt_codec_tests(void)
{
    RUN_TEST(test_is_gatt_request);
    RUN_TEST(test_decode_scan_start);
    RUN_TEST(test_decode_connect);
    RUN_TEST(test_decode_conn_id_requests);
    RUN_TEST(test_decode_read);
    RUN_TEST(test_decode_write);
    RUN_TEST(test_decode_subscribe);
    RUN_TEST(test_decode_set_failsafe);
    RUN_TEST(test_encode_responses);
    RUN_TEST(test_encode_events);
    RUN_TEST(test_encode_limits);
    RUN_TEST(test_hello_rsp_max_conns_range);
}
