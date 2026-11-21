/**
 * @file bc2_usb.h
 * @brief Byte transport over the ESP32-S3 USB Serial/JTAG peripheral, with a TX queue.
 *
 * The peripheral has a fixed USB identity (VID 0x303A, PID 0x1001), so the firmware is identified
 * by the HELLO_RSP magic, not by a product string (decision of the product owner).
 *
 * Transmit path: producers (bc2_core under its mutex, the NimBLE side) call bc2_usb_enqueue(),
 * which never blocks. A dedicated TX task writes the queued frames to the port; if the host does
 * not read, only that task waits, never the BLE stack or the safety timers.
 */
#ifndef BC2_USB_H
#define BC2_USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/** Install the USB Serial/JTAG driver (1 KiB RX/TX buffers) and start the TX task with its 3 KiB frame queue. */
esp_err_t bc2_usb_init(void);

/**
 * Read up to @p max bytes, waiting at most @p timeout_ms.
 * @return number of bytes read (0 on timeout)
 */
size_t bc2_usb_read(uint8_t *buf, size_t max, uint32_t timeout_ms);

/**
 * Queue one complete frame for transmission; never blocks.
 * @param droppable true for scan reports and notifications (refused when the queue is more than half full)
 * @return false if the frame was refused (queue full or pressure); the queue counts the drop
 */
bool bc2_usb_enqueue(const uint8_t *data, size_t len, bool droppable);

#endif /* BC2_USB_H */
