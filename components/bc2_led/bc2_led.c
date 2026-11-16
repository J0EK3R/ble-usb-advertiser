/**
 * @file bc2_led.c
 * @brief LED pattern engine and WS2812 output (patterns are documented in README.md).
 */
#include "bc2_led.h"

#include "board.h"
#include "led_strip.h"
#include "sdkconfig.h"

/** Handle of the LED strip driver (one LED). */
static led_strip_handle_t s_strip;

/** Blink helper: true during the first @p on_ms of every @p period_ms. */
static bool blink(uint32_t now_ms, uint32_t period_ms, uint32_t on_ms)
{
    return (now_ms % period_ms) < on_ms;
}

bc2_rgb_t bc2_led_pattern(bc2_led_state_t state, bool flash, uint32_t now_ms)
{
    const bc2_rgb_t off = {0, 0, 0};
    switch (state) {
    case BC2_LED_BOOT:
        return (bc2_rgb_t){255, 255, 255};                               /* white, solid */
    case BC2_LED_NO_HOST:
        return blink(now_ms, 1000, 500) ? (bc2_rgb_t){0, 0, 255} : off;  /* blue, 1 Hz */
    case BC2_LED_CONNECTED:
        return (bc2_rgb_t){0, 255, 0};                                   /* green, solid */
    case BC2_LED_ADVERTISING: {
        if (flash) {
            return (bc2_rgb_t){255, 255, 255};                           /* white flash per update */
        }
        const uint32_t phase = now_ms % 1000;                            /* triangle wave, 1 s period */
        const uint32_t tri = phase < 500 ? phase : 1000 - phase;         /* 0..500 */
        const uint8_t level = (uint8_t)(64 + (tri * 191) / 500);         /* 64..255, never fully dark */
        return (bc2_rgb_t){0, level, level};                             /* cyan pulse */
    }
    case BC2_LED_ERROR:
        return blink(now_ms, 250, 125) ? (bc2_rgb_t){255, 0, 0} : off;   /* red, 4 Hz */
    case BC2_LED_TIMEOUT:
        return (bc2_rgb_t){255, 100, 0};                                 /* orange, solid */
    case BC2_LED_BUTTON_STOP:
        return (bc2_rgb_t){255, 0, 0};                                   /* red flash */
    }
    return off;
}

esp_err_t bc2_led_init(void)
{
    const led_strip_config_t strip_cfg = {
        .strip_gpio_num = BC2_BOARD_LED_GPIO,
        .max_leds = BC2_BOARD_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    const led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    const esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (err != ESP_OK) {
        return err;
    }
    return led_strip_clear(s_strip);
}

void bc2_led_show(bc2_rgb_t color)
{
    /* scale to the configured brightness, the WS2812 is blinding at full power */
    const uint32_t k = CONFIG_BC2ADV_LED_BRIGHTNESS;
    led_strip_set_pixel(s_strip, 0, (color.r * k) / 255, (color.g * k) / 255, (color.b * k) / 255);
    led_strip_refresh(s_strip);
}
