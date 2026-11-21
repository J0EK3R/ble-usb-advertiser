/**
 * @file bc2_usb.c
 * @brief USB Serial/JTAG backend of the byte transport: RX read and a non blocking TX queue with its own task.
 */
#include "bc2_usb.h"

#include "bc2_txq.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** Maximum time a write may block when the host does not read (milliseconds). */
#define WRITE_TIMEOUT_MS 20
/** Size of the frame queue: about 50 notifications; droppable frames may use half of it. */
#define TX_QUEUE_BYTES 3072
/** Priority of the TX task: below the USB RX and core tasks, above the LED task. */
#define TX_TASK_PRIORITY 4

static uint8_t s_queue_storage[TX_QUEUE_BYTES];
static bc2_txq_t s_queue;
static portMUX_TYPE s_queue_mux = portMUX_INITIALIZER_UNLOCKED;   /* guards s_queue (short memcpy only) */
static TaskHandle_t s_tx_task;

/** Frames lost or truncated because the host did not read (only touched by the TX task). */
static uint32_t s_tx_lost_frames;

/**
 * Write a frame to the port. While the host does not read, the write times out after WRITE_TIMEOUT_MS and the
 * rest of the frame is dropped (the host sees a truncated frame, rejected by LEN/CRC). Such losses are counted and
 * logged (rate limited); the frozen GATT_STATUS_RSP layout has no field for them.
 */
static void write_frame(const uint8_t *data, size_t len)
{
    const int written = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
    if (written < 0 || (size_t)written < len) {
        s_tx_lost_frames++;
        if ((s_tx_lost_frames & (s_tx_lost_frames - 1)) == 0) {   /* log at 1, 2, 4, 8 ... only */
            ESP_LOGW("bc2_usb", "host not reading: %u frame(s) lost or truncated", (unsigned)s_tx_lost_frames);
        }
    }
}

/** TX task: wait for queued frames and write them out. The only place that may block on the host. */
static void tx_task(void *param)
{
    (void)param;
    uint8_t frame[BC2_TXQ_MAX_FRAME];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                  /* woken by bc2_usb_enqueue() */
        for (;;) {
            portENTER_CRITICAL(&s_queue_mux);
            const size_t n = bc2_txq_pop(&s_queue, frame);
            portEXIT_CRITICAL(&s_queue_mux);
            if (n == 0) {
                break;
            }
            write_frame(frame, n);
        }
    }
}

esp_err_t bc2_usb_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 1024;
    cfg.tx_buffer_size = 1024;
    const esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    bc2_txq_init(&s_queue, s_queue_storage, sizeof(s_queue_storage));
    return xTaskCreate(tx_task, "bc2_tx", 3072, NULL, TX_TASK_PRIORITY, &s_tx_task) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

size_t bc2_usb_read(uint8_t *buf, size_t max, uint32_t timeout_ms)
{
    const int n = usb_serial_jtag_read_bytes(buf, (uint32_t)max, pdMS_TO_TICKS(timeout_ms));
    return n > 0 ? (size_t)n : 0;
}

bool bc2_usb_enqueue(const uint8_t *data, size_t len, bool droppable)
{
    portENTER_CRITICAL(&s_queue_mux);
    const bool queued = bc2_txq_push(&s_queue, data, len, droppable);
    portEXIT_CRITICAL(&s_queue_mux);
    if (queued && s_tx_task != NULL) {
        xTaskNotifyGive(s_tx_task);
    }
    return queued;
}
