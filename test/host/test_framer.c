/**
 * @file test_framer.c
 * @brief Tests for the frame encoder and the streaming parser (resync, timeout, limits).
 */
#include "bc2_protocol.h"
#include "test_minunit.h"
#include "test_vectors.h"

/** Feed bytes at a constant time and collect frames; returns the number of frames found. */
static int feed(bc2_framer_t *f, const uint8_t *data, size_t len, uint32_t now_ms, bc2_frame_t *frames,
                int max_frames)
{
    int n = 0;
    for (size_t i = 0; i < len; i++) {
        bc2_framer_push(f, data[i], now_ms);
        bc2_frame_t fr;
        while (bc2_framer_next(f, &fr)) {
            if (n < max_frames) {
                frames[n] = fr;
            }
            n++;
        }
    }
    return n;
}

/** A frame arriving in one chunk is parsed with the right fields. */
static void test_framer_single_chunk(void)
{
    bc2_framer_t f;
    bc2_frame_t fr[2];
    bc2_framer_init(&f);
    ASSERT_EQ(1, feed(&f, GV_SET_UPDATE, sizeof(GV_SET_UPDATE), 0, fr, 2));
    ASSERT_EQ(BC2_MSG_SET_UPDATE, fr[0].type);
    ASSERT_EQ(4, fr[0].seq);
    ASSERT_EQ(8, fr[0].len);
    ASSERT_EQ(0xA4, fr[0].payload[7]);
    ASSERT_EQ(0, f.parse_errors);
}

/** Byte by byte with 1 ms spacing gives the same result. */
static void test_framer_byte_by_byte(void)
{
    bc2_framer_t f;
    bc2_frame_t fr;
    int n = 0;
    bc2_framer_init(&f);
    for (size_t i = 0; i < sizeof(GV_HELLO); i++) {
        bc2_framer_push(&f, GV_HELLO[i], (uint32_t)i);
        while (bc2_framer_next(&f, &fr)) {
            n++;
        }
    }
    ASSERT_EQ(1, n);
    ASSERT_EQ(BC2_MSG_HELLO, fr.type);
}

/** Two frames back to back in one buffer. */
static void test_framer_back_to_back(void)
{
    uint8_t buf[64];
    bc2_framer_t f;
    bc2_frame_t fr[4];
    bc2_framer_init(&f);
    memcpy(buf, GV_PING, sizeof(GV_PING));
    memcpy(buf + sizeof(GV_PING), GV_STOP_ALL, sizeof(GV_STOP_ALL));
    ASSERT_EQ(2, feed(&f, buf, sizeof(GV_PING) + sizeof(GV_STOP_ALL), 0, fr, 4));
    ASSERT_EQ(BC2_MSG_PING, fr[0].type);
    ASSERT_EQ(BC2_MSG_STOP_ALL, fr[1].type);
}

/** Garbage (including text from the ROM bootloader) before a SOF is skipped without errors. */
static void test_framer_garbage_before_sof(void)
{
    uint8_t buf[64] = {'r', 'e', 's', 'e', 't', '!', 0x00, 0xFF};
    bc2_framer_t f;
    bc2_frame_t fr[2];
    bc2_framer_init(&f);
    memcpy(buf + 8, GV_PING, sizeof(GV_PING));
    ASSERT_EQ(1, feed(&f, buf, 8 + sizeof(GV_PING), 0, fr, 2));
    ASSERT_EQ(0, f.parse_errors);
}

/** A SOF byte value inside the payload does not break parsing. */
static void test_framer_sof_in_payload(void)
{
    bc2_frame_t in = {.type = BC2_MSG_SET_UPDATE, .seq = 9, .len = 4,
                      .payload = {0, 0xB2, 0xB2, 0x00}};
    uint8_t wire[BC2_MAX_FRAME];
    bc2_framer_t f;
    bc2_frame_t fr[2];
    const size_t n = bc2_frame_encode(&in, wire, sizeof(wire));
    bc2_framer_init(&f);
    ASSERT_EQ(1, feed(&f, wire, n, 0, fr, 2));
    ASSERT_EQ(0xB2, fr[0].payload[1]);
}

/** A corrupted frame is dropped and counted; a valid frame inside its bytes is still found. */
static void test_framer_resync_inside_corrupted(void)
{
    /* Corrupted SET_START (CRC broken) whose payload bytes contain a complete valid PING. */
    uint8_t buf[80];
    size_t n = 0;
    bc2_framer_t f;
    bc2_frame_t fr[4];
    bc2_frame_t outer = {.type = BC2_MSG_SET_UPDATE, .seq = 1, .len = 0};
    bc2_framer_init(&f);
    memcpy(outer.payload, GV_PING, sizeof(GV_PING));
    outer.len = sizeof(GV_PING);
    n = bc2_frame_encode(&outer, buf, sizeof(buf));
    buf[n - 1] ^= 0xFF;                                  /* destroy the outer CRC */
    ASSERT_EQ(1, feed(&f, buf, n, 0, fr, 4));
    ASSERT_EQ(BC2_MSG_PING, fr[0].type);
    ASSERT_EQ(1, f.parse_errors);
}

/** A bad CRC alone counts one error and yields no frame; the next valid frame still works. */
static void test_framer_bad_crc_then_valid(void)
{
    uint8_t bad[sizeof(GV_PING)];
    bc2_framer_t f;
    bc2_frame_t fr[2];
    memcpy(bad, GV_PING, sizeof(bad));
    bad[sizeof(bad) - 1] ^= 0x01;
    bc2_framer_init(&f);
    ASSERT_EQ(0, feed(&f, bad, sizeof(bad), 0, fr, 2));
    ASSERT_EQ(1, f.parse_errors);
    ASSERT_EQ(1, feed(&f, GV_PING, sizeof(GV_PING), 1, fr, 2));
}

/** LEN values 0, 1, 63 and 255 are rejected (count an error, no frame). */
static void test_framer_invalid_len(void)
{
    const uint8_t lens[] = {0, 1, 63, 255};
    for (size_t i = 0; i < sizeof(lens); i++) {
        bc2_framer_t f;
        bc2_frame_t fr[2];
        uint8_t buf[4] = {BC2_SOF, lens[i], 0x02, 0x00};
        bc2_framer_init(&f);
        ASSERT_EQ(0, feed(&f, buf, sizeof(buf), 0, fr, 2));
        ASSERT_TRUE(f.parse_errors >= 1);
    }
}

/** The largest frame (60 byte payload, 66 bytes on the wire) round-trips. */
static void test_framer_max_frame(void)
{
    bc2_frame_t in = {.type = BC2_MSG_ACK, .seq = 77, .len = BC2_MAX_PAYLOAD};
    uint8_t wire[BC2_MAX_FRAME];
    bc2_framer_t f;
    bc2_frame_t fr[2];
    for (size_t i = 0; i < BC2_MAX_PAYLOAD; i++) {
        in.payload[i] = (uint8_t)i;
    }
    ASSERT_EQ(BC2_MAX_FRAME, bc2_frame_encode(&in, wire, sizeof(wire)));
    bc2_framer_init(&f);
    ASSERT_EQ(1, feed(&f, wire, BC2_MAX_FRAME, 0, fr, 2));
    ASSERT_EQ(BC2_MAX_PAYLOAD, fr[0].len);
    ASSERT_MEM_EQ(in.payload, fr[0].payload, BC2_MAX_PAYLOAD);
}

/** A truncated frame followed by 200+ ms silence is dropped and does not eat the next frame. */
static void test_framer_interframe_timeout(void)
{
    bc2_framer_t f;
    bc2_frame_t fr[2];
    bc2_framer_init(&f);
    ASSERT_EQ(0, feed(&f, GV_SET_START, 10, 0, fr, 2));                    /* half a frame */
    ASSERT_EQ(1, feed(&f, GV_PING, sizeof(GV_PING), 300, fr, 2));          /* 300 ms later */
    ASSERT_EQ(BC2_MSG_PING, fr[0].type);
    ASSERT_EQ(1, f.parse_errors);
}

/** Without the timeout the same stale bytes would swallow the following frame (sanity check). */
static void test_framer_no_timeout_within_window(void)
{
    bc2_framer_t f;
    bc2_frame_t fr[2];
    bc2_framer_init(&f);
    ASSERT_EQ(0, feed(&f, GV_SET_START, 10, 0, fr, 2));
    ASSERT_EQ(1, feed(&f, GV_SET_START + 10, sizeof(GV_SET_START) - 10, 150, fr, 2));
    ASSERT_EQ(BC2_MSG_SET_START, fr[0].type);
    ASSERT_EQ(0, f.parse_errors);
}

/** Encoder rejects oversize frames and too small buffers; output matches the golden vector. */
static void test_encoder(void)
{
    uint8_t out[BC2_MAX_FRAME];
    bc2_frame_t fr = {.type = BC2_MSG_STOP_ALL, .seq = 6, .len = 0};
    ASSERT_EQ(sizeof(GV_STOP_ALL), bc2_frame_encode(&fr, out, sizeof(out)));
    ASSERT_MEM_EQ(GV_STOP_ALL, out, sizeof(GV_STOP_ALL));
    ASSERT_EQ(0, bc2_frame_encode(&fr, out, 5));
    fr.len = BC2_MAX_PAYLOAD + 1;
    ASSERT_EQ(0, bc2_frame_encode(&fr, out, sizeof(out)));
}

void run_framer_tests(void)
{
    RUN_TEST(test_framer_single_chunk);
    RUN_TEST(test_framer_byte_by_byte);
    RUN_TEST(test_framer_back_to_back);
    RUN_TEST(test_framer_garbage_before_sof);
    RUN_TEST(test_framer_sof_in_payload);
    RUN_TEST(test_framer_resync_inside_corrupted);
    RUN_TEST(test_framer_bad_crc_then_valid);
    RUN_TEST(test_framer_invalid_len);
    RUN_TEST(test_framer_max_frame);
    RUN_TEST(test_framer_interframe_timeout);
    RUN_TEST(test_framer_no_timeout_within_window);
    RUN_TEST(test_encoder);
}
