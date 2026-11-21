/**
 * @file main.c
 * @brief Firmware entry point: creates the core, wires the hardware ops and starts the tasks.
 *
 * Tasks:
 *   usb  (prio 6) reads USB bytes and feeds bc2_core
 *   core (prio 5) housekeeping every 20 ms or when the GATT adapter queued an event: keep-alive
 *                 timeout, GATT events and deadlines, async set failures, button
 *   tx   (prio 4) in bc2_usb: writes the queued frames to the USB port (may block on the host)
 *   led  (prio 3) 50 ms LED pattern update
 *   NimBLE host task (created by bc2_ble) runs the BLE callbacks and only queues events
 * All bc2_core calls are serialised by s_core_mutex because the core itself is not thread safe.
 * Nothing that blocks on the host (USB writes) runs under that mutex any more: bc2_core only
 * enqueues its frames (bc2_usb_enqueue).
 */
#include <stdbool.h>
#include <stdint.h>

#include "bc2_ble.h"
#include "bc2_core.h"
#include "sdkconfig.h"
#if CONFIG_BC2ADV_ENABLE_GATT
#include "bc2_gatt_nimble.h"
#endif
#include "bc2_led.h"
#include "bc2_usb.h"
#include "board.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/** Longest wait of the core housekeeping task in milliseconds (keep-alive and deadlines are checked at least this often). */
#define CORE_PERIOD_MS 20
/** Period of the LED task in milliseconds. */
#define LED_PERIOD_MS 50
/** USB read chunk size and wait time. */
#define USB_READ_CHUNK 64
#define USB_READ_WAIT_MS 20

static bc2_core_t s_core;
static SemaphoreHandle_t s_core_mutex;
static TaskHandle_t s_core_task;          /**< woken by the GATT adapter when it queued an event */

/** Milliseconds since boot (wraps after 49 days, all core arithmetic is wrap safe). */
static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ---- bc2_io_ops_t ---- */

/*
 * The core calls these while s_core_mutex is held. They only queue the frame for the TX task and
 * never block, so a host that stops reading cannot delay the keep-alive tick, the button or the
 * safety stop. A full queue drops frames (counted by the queue).
 */
static void io_write(void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    bc2_usb_enqueue(data, len, false);
}

/** Scan reports and notifications: refused when the queue is under pressure. */
static bool io_write_droppable(void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    return bc2_usb_enqueue(data, len, true);
}

static uint32_t io_now_us(void *ctx)
{
    (void)ctx;
    return (uint32_t)esp_timer_get_time();
}

/** Emergency reset: the only way to be sure that the radio is silent after a failed stop. */
static void io_hard_reset(void *ctx)
{
    (void)ctx;
    esp_restart();
}

/* ---- tasks ---- */

/** Read USB bytes and let the core parse and execute them. */
static void usb_task(void *param)
{
    (void)param;
    uint8_t buf[USB_READ_CHUNK];
    esp_task_wdt_add(NULL);                                  /* a hang here must reset the chip */
    for (;;) {
        const size_t n = bc2_usb_read(buf, sizeof(buf), USB_READ_WAIT_MS);
        if (n > 0) {
            xSemaphoreTake(s_core_mutex, portMAX_DELAY);
            bc2_core_on_bytes(&s_core, buf, n, now_ms());
            xSemaphoreGive(s_core_mutex);
        }
        esp_task_wdt_reset();
    }
}

/**
 * Debounced button edge detector; returns true once per press. Time based (not per call), because the
 * core task now also runs between its 20 ms ticks when the GATT adapter wakes it.
 */
static bool button_pressed_event(uint32_t t_ms)
{
    static uint32_t low_since_ms;
    static bool low;
    static bool fired;
    if (gpio_get_level(BC2_BOARD_BUTTON_GPIO) == 0) {        /* active low */
        if (!low) {
            low = true;
            low_since_ms = t_ms;
        }
    } else {
        low = false;
        fired = false;
    }
    if (low && !fired && (uint32_t)(t_ms - low_since_ms) >= BC2_BOARD_BUTTON_DEBOUNCE_MS) {
        fired = true;
        return true;
    }
    return false;
}

/** Called by the GATT adapter (NimBLE host task) after it queued an event: process it right away. */
static void wake_core_task(void)
{
    if (s_core_task != NULL) {
        xTaskNotifyGive(s_core_task);
    }
}

/** Keep-alive supervision, GATT events and deadlines, asynchronous failures and the emergency button. */
static void core_task(void *param)
{
    (void)param;
    esp_task_wdt_add(NULL);
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CORE_PERIOD_MS));   /* event wake-up or 20 ms tick */
        xSemaphoreTake(s_core_mutex, portMAX_DELAY);
        const uint32_t t = now_ms();
        bc2_core_tick(&s_core, t);
        if (button_pressed_event(t)) {
            bc2_core_on_button(&s_core, t);
        }
        xSemaphoreGive(s_core_mutex);
        esp_task_wdt_reset();
    }
}

/** Show the state of the core on the RGB LED. */
static void led_task(void *param)
{
    (void)param;
    for (;;) {
        xSemaphoreTake(s_core_mutex, portMAX_DELAY);
        const uint32_t t = now_ms();
        const bc2_led_state_t state = bc2_core_led_state(&s_core, t);
        const bool flash = bc2_core_activity_flash(&s_core, t);
        xSemaphoreGive(s_core_mutex);
        bc2_led_show(bc2_led_pattern(state, flash, t));
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

/** Configure the button GPIO as input with pull-up. */
static void button_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BC2_BOARD_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&cfg);
}

void app_main(void)
{
    s_core_mutex = xSemaphoreCreateMutex();
    bc2_led_init();
    bc2_led_show(bc2_led_pattern(BC2_LED_BOOT, false, 0));   /* white flash while booting */
    button_init();

    bc2_ble_init();
    bc2_usb_init();
    const bc2_gatt_ops_t *gatt_ops = NULL;                   /* NULL: build without GATT */
#if CONFIG_BC2ADV_ENABLE_GATT
    if (bc2_gatt_nimble_init(wake_core_task) == ESP_OK) {
        gatt_ops = bc2_gatt_nimble_ops();
    }
#endif
    const bc2_io_ops_t io = {.ctx = NULL, .write = io_write, .write_droppable = io_write_droppable,
                             .now_us = io_now_us, .hard_reset = io_hard_reset};
    bc2_core_init(&s_core, bc2_ble_ops(), gatt_ops, &io);
    vTaskDelay(pdMS_TO_TICKS(200));                          /* make the boot flash visible */

    xTaskCreate(usb_task, "bc2_usb", 5120, NULL, 6, NULL);
    xTaskCreate(core_task, "bc2_core", 6144, NULL, 5, &s_core_task);
    xTaskCreate(led_task, "bc2_led", 3072, NULL, 3, NULL);
}
