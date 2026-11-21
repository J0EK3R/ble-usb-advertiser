/**
 * @file bc2_txq.h
 * @brief Byte ring buffer holding complete outgoing frames, with a drop policy.
 *
 * Pure C11, not thread safe (the caller serialises push/pop, see main.c). Producers (the core,
 * which runs under its mutex, and the NimBLE side) never block: a frame that does not fit is
 * rejected and counted. Two classes exist:
 *   - critical:  responses and state events, rejected only when the buffer is completely full;
 *   - droppable: scan reports and notifications, rejected once the buffer is more than half full,
 *                so a burst of reports can never push out the critical frames.
 * One consumer task pops frames and writes them to the USB port, which may block for 20 ms per
 * frame without delaying the BLE stack or the safety timers.
 */
#ifndef BC2_TXQ_H
#define BC2_TXQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Largest frame that can be queued (BC2_MAX_FRAME of the protocol). */
#define BC2_TXQ_MAX_FRAME 66u

/** Queue state. Treat as opaque; initialise with bc2_txq_init(). */
typedef struct {
    uint8_t *buf;                 /**< storage provided by the caller */
    uint16_t cap;                 /**< capacity of @c buf in bytes */
    uint16_t head;                /**< read position */
    uint16_t tail;                /**< write position */
    uint16_t used;                /**< occupied bytes (including 1 length byte per frame) */
    uint16_t frames;              /**< number of queued frames */
    uint32_t dropped_droppable;   /**< droppable frames rejected (total) */
    uint32_t dropped_critical;    /**< critical frames rejected because the buffer was full (total) */
} bc2_txq_t;

/** Initialise a queue on a caller provided buffer of @p cap bytes (cap <= 65535). */
void bc2_txq_init(bc2_txq_t *q, uint8_t *buf, uint16_t cap);

/**
 * Queue one frame.
 * @param droppable true for scan reports / notifications (rejected above half fill level)
 * @return true if queued, false if rejected (counted in the matching drop counter)
 */
bool bc2_txq_push(bc2_txq_t *q, const uint8_t *data, size_t len, bool droppable);

/**
 * Remove the oldest frame.
 * @param out buffer of at least BC2_TXQ_MAX_FRAME bytes
 * @return frame length, 0 if the queue is empty
 */
size_t bc2_txq_pop(bc2_txq_t *q, uint8_t *out);

/** Number of queued frames. */
uint16_t bc2_txq_frames(const bc2_txq_t *q);

/** Occupied bytes. */
uint16_t bc2_txq_used(const bc2_txq_t *q);

#ifdef __cplusplus
}
#endif

#endif /* BC2_TXQ_H */
