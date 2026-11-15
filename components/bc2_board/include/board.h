/**
 * @file board.h
 * @brief Pin assignment of the M5Stack AtomS3U (ESP32-S3FN8).
 *
 * All board specific values live here. UNVERIFIED: the pins were taken from memory, not from the
 * schematic. Check them against the M5Stack AtomS3U schematic when the hardware arrives
 * (see docs/hardware.md and docs/verification.md item 12).
 */
#ifndef BC2_BOARD_H
#define BC2_BOARD_H

/** GPIO of the single WS2812 RGB LED data line. */
#define BC2_BOARD_LED_GPIO 35
/** Number of LEDs on the data line. */
#define BC2_BOARD_LED_COUNT 1
/** GPIO of the user button (active low, external pull-up on the board, internal one enabled too). */
#define BC2_BOARD_BUTTON_GPIO 41
/** Button debounce time in milliseconds. */
#define BC2_BOARD_BUTTON_DEBOUNCE_MS 30

#endif /* BC2_BOARD_H */
