/**
 * @file bc2_txq.c
 * @brief Implementation of the frame ring buffer, see bc2_txq.h.
 *
 * Storage layout: [len][frame bytes] entries back to back, wrapping around the end of the buffer.
 */
#include "bc2_txq.h"

/** Copy @p len bytes into the ring starting at @p pos (wraps) and return the new position. */
static uint16_t ring_write(bc2_txq_t *q, uint16_t pos, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        q->buf[pos] = src[i];
        pos = (uint16_t)(pos + 1u == q->cap ? 0u : pos + 1u);   /* wrap at the end of the storage */
    }
    return pos;
}

/** Copy @p len bytes out of the ring starting at @p pos (wraps) and return the new position. */
static uint16_t ring_read(const bc2_txq_t *q, uint16_t pos, uint8_t *dst, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        dst[i] = q->buf[pos];
        pos = (uint16_t)(pos + 1u == q->cap ? 0u : pos + 1u);
    }
    return pos;
}

void bc2_txq_init(bc2_txq_t *q, uint8_t *buf, uint16_t cap)
{
    q->buf = buf;
    q->cap = cap;
    q->head = 0;
    q->tail = 0;
    q->used = 0;
    q->frames = 0;
    q->dropped_droppable = 0;
    q->dropped_critical = 0;
}

bool bc2_txq_push(bc2_txq_t *q, const uint8_t *data, size_t len, bool droppable)
{
    const size_t need = len + 1u;                           /* frame + length byte */
    /* Droppable frames may only use the first half of the storage, critical ones all of it. */
    const size_t limit = droppable ? (size_t)q->cap / 2u : (size_t)q->cap;
    if (len == 0 || len > BC2_TXQ_MAX_FRAME || (size_t)q->used + need > limit) {
        if (droppable) {
            q->dropped_droppable++;
        } else {
            q->dropped_critical++;
        }
        return false;
    }
    const uint8_t len_byte = (uint8_t)len;
    q->tail = ring_write(q, q->tail, &len_byte, 1);
    q->tail = ring_write(q, q->tail, data, len);
    q->used = (uint16_t)(q->used + need);
    q->frames++;
    return true;
}

size_t bc2_txq_pop(bc2_txq_t *q, uint8_t *out)
{
    if (q->frames == 0) {
        return 0;
    }
    uint8_t len = 0;
    q->head = ring_read(q, q->head, &len, 1);
    q->head = ring_read(q, q->head, out, len);
    q->used = (uint16_t)(q->used - (len + 1u));
    q->frames--;
    return len;
}

uint16_t bc2_txq_frames(const bc2_txq_t *q)
{
    return q->frames;
}

uint16_t bc2_txq_used(const bc2_txq_t *q)
{
    return q->used;
}
