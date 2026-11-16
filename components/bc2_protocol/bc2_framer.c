/**
 * @file bc2_framer.c
 * @brief Frame encoder and streaming parser with resynchronisation.
 *
 * Frame layout: SOF(0xB2) LEN TYPE SEQ PAYLOAD CRC16-LE, where LEN counts TYPE+SEQ+PAYLOAD and
 * the CRC covers LEN..PAYLOAD. See docs/protocol.md section 2.
 */
#include <string.h>

#include "bc2_protocol.h"

size_t bc2_frame_encode(const bc2_frame_t *frame, uint8_t *out, size_t cap)
{
    if (frame->len > BC2_MAX_PAYLOAD) {
        return 0;
    }
    const size_t total = (size_t)frame->len + 6u;     /* SOF + LEN + TYPE + SEQ + payload + CRC(2) */
    if (cap < total) {
        return 0;
    }
    out[0] = BC2_SOF;
    out[1] = (uint8_t)(frame->len + 2u);               /* LEN = TYPE + SEQ + payload */
    out[2] = frame->type;
    out[3] = frame->seq;
    memcpy(&out[4], frame->payload, frame->len);
    const uint16_t crc = bc2_crc16(&out[1], (size_t)frame->len + 3u);   /* LEN..payload */
    out[4 + frame->len] = (uint8_t)(crc & 0xFFu);      /* little endian */
    out[5 + frame->len] = (uint8_t)(crc >> 8);
    return total;
}

void bc2_framer_init(bc2_framer_t *f)
{
    memset(f, 0, sizeof(*f));
}

/** Remove the first @p n bytes from the buffer. */
static void framer_drop(bc2_framer_t *f, size_t n)
{
    memmove(f->buf, &f->buf[n], f->len - n);
    f->len -= n;
}

void bc2_framer_push(bc2_framer_t *f, uint8_t byte, uint32_t now_ms)
{
    /* A stalled partial frame must not swallow the next valid frame: drop it after the timeout. */
    if (f->len > 0 && (uint32_t)(now_ms - f->start_ms) > BC2_FRAME_TIMEOUT_MS) {
        f->len = 0;
        f->parse_errors++;
    }
    if (f->len == 0) {
        f->start_ms = now_ms;
    }
    if (f->len == sizeof(f->buf)) {
        /* Cannot happen while bc2_framer_next() is called after every push; stay safe anyway. */
        framer_drop(f, 1);
    }
    f->buf[f->len++] = byte;
}

bool bc2_framer_next(bc2_framer_t *f, bc2_frame_t *out)
{
    while (f->len > 0) {
        if (f->buf[0] != BC2_SOF) {
            /* Skip garbage up to the next SOF candidate (e.g. ROM boot text); not an error. */
            size_t skip = 1;
            while (skip < f->len && f->buf[skip] != BC2_SOF) {
                skip++;
            }
            framer_drop(f, skip);
            continue;
        }
        if (f->len < 2) {
            return false;                                  /* LEN not received yet */
        }
        const uint8_t len = f->buf[1];
        if (len < BC2_MIN_LEN || len > BC2_MAX_LEN) {
            framer_drop(f, 1);                             /* impossible LEN: this SOF was no frame start */
            f->parse_errors++;
            continue;
        }
        const size_t total = (size_t)len + 4u;             /* SOF + LEN + (TYPE..PAYLOAD) + CRC(2) */
        if (f->len < total) {
            return false;                                  /* wait for the rest */
        }
        const uint16_t crc_rx = (uint16_t)(f->buf[total - 2] | (f->buf[total - 1] << 8));
        if (bc2_crc16(&f->buf[1], (size_t)len + 1u) != crc_rx) {
            framer_drop(f, 1);                             /* only the SOF: rescan the following bytes */
            f->parse_errors++;
            continue;
        }
        out->type = f->buf[2];
        out->seq = f->buf[3];
        out->len = (uint8_t)(len - 2u);
        memcpy(out->payload, &f->buf[4], out->len);
        framer_drop(f, total);
        return true;
    }
    return false;
}
