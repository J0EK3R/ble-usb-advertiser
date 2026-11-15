/**
 * @file test_crc.c
 * @brief Tests for the CRC-16/CCITT-FALSE implementation.
 */
#include "bc2_protocol.h"
#include "test_minunit.h"

/** The standard check value of CRC-16/CCITT-FALSE for the ASCII string "123456789". */
static void test_crc_check_value(void)
{
    ASSERT_EQ(0x29B1, bc2_crc16((const uint8_t *)"123456789", 9));
}

/** An empty input yields the initial value. */
static void test_crc_empty(void)
{
    ASSERT_EQ(0xFFFF, bc2_crc16(NULL, 0));
}

/** Flipping any single bit changes the CRC. */
static void test_crc_bit_flip(void)
{
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint16_t ref = bc2_crc16(data, sizeof(data));
    for (size_t i = 0; i < sizeof(data); i++) {
        for (int bit = 0; bit < 8; bit++) {
            data[i] ^= (uint8_t)(1u << bit);
            ASSERT_TRUE(bc2_crc16(data, sizeof(data)) != ref);
            data[i] ^= (uint8_t)(1u << bit);
        }
    }
}

/** Incremental update equals one-shot computation. */
static void test_crc_incremental(void)
{
    const uint8_t data[] = "123456789";
    uint16_t crc = BC2_CRC_INIT;
    crc = bc2_crc16_update(crc, data, 4);
    crc = bc2_crc16_update(crc, data + 4, 5);
    ASSERT_EQ(0x29B1, crc);
}

void run_crc_tests(void)
{
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_crc_empty);
    RUN_TEST(test_crc_bit_flip);
    RUN_TEST(test_crc_incremental);
}
