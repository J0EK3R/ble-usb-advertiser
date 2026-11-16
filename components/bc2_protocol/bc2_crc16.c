/**
 * @file bc2_crc16.c
 * @brief CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
 */
#include "bc2_protocol.h"

uint16_t bc2_crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);   /* bring the next byte into the high half */
        for (int bit = 0; bit < 8; bit++) {
            /* shift left; if the bit that fell out was set, apply the polynomial */
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t bc2_crc16(const uint8_t *data, size_t len)
{
    return bc2_crc16_update(BC2_CRC_INIT, data, len);
}
