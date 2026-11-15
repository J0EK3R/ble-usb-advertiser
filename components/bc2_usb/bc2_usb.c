/**
 * @file bc2_usb.c
 * @brief USB Serial/JTAG backend of the byte transport.
 */
#include "bc2_usb.h"

#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"

/** Maximum time a write may block when the host does not read (milliseconds). */
#define WRITE_TIMEOUT_MS 20

esp_err_t bc2_usb_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 1024;
    cfg.tx_buffer_size = 1024;
    return usb_serial_jtag_driver_install(&cfg);
}

size_t bc2_usb_read(uint8_t *buf, size_t max, uint32_t timeout_ms)
{
    const int n = usb_serial_jtag_read_bytes(buf, (uint32_t)max, pdMS_TO_TICKS(timeout_ms));
    return n > 0 ? (size_t)n : 0;
}

void bc2_usb_write(const uint8_t *data, size_t len)
{
    /* The return value (bytes queued) is ignored on purpose: a partial write corrupts one frame,
     * which the host detects through the CRC / response timeout. */
    (void)usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
}
