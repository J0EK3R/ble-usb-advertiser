/**
 * @file bc2_usb.h
 * @brief Byte transport over the ESP32-S3 USB Serial/JTAG peripheral.
 *
 * The peripheral has a fixed USB identity (VID 0x303A, PID 0x1001), so the firmware is identified
 * by the HELLO_RSP magic, not by a product string (decision of the product owner).
 */
#ifndef BC2_USB_H
#define BC2_USB_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/** Install the USB Serial/JTAG driver with 1 KiB RX and TX buffers. */
esp_err_t bc2_usb_init(void);

/**
 * Read up to @p max bytes, waiting at most @p timeout_ms.
 * @return number of bytes read (0 on timeout)
 */
size_t bc2_usb_read(uint8_t *buf, size_t max, uint32_t timeout_ms);

/**
 * Write bytes with a short timeout (20 ms). If the host does not read, the data is dropped:
 * the safety logic must never depend on the host draining the port.
 */
void bc2_usb_write(const uint8_t *data, size_t len);

#endif /* BC2_USB_H */
