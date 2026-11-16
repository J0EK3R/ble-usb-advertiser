/**
 * @file bc2_led.h
 * @brief Maps bc2_core LED states to colours/blink patterns on the WS2812 LED.
 */
#ifndef BC2_LED_H
#define BC2_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "bc2_core.h"
#include "esp_err.h"

/** Colour of one LED update, 0..255 per channel before brightness scaling. */
typedef struct {
    uint8_t r, g, b;
} bc2_rgb_t;

/**
 * Compute the colour for a state at a given time (pure function).
 * @param state  LED state from bc2_core_led_state()
 * @param flash  white activity flash from bc2_core_activity_flash()
 * @param now_ms millisecond clock driving the blink phase
 */
bc2_rgb_t bc2_led_pattern(bc2_led_state_t state, bool flash, uint32_t now_ms);

/** Initialise the LED driver (RMT) and switch the LED off. */
esp_err_t bc2_led_init(void);

/** Show a colour, scaled by CONFIG_BC2ADV_LED_BRIGHTNESS. */
void bc2_led_show(bc2_rgb_t color);

#endif /* BC2_LED_H */
