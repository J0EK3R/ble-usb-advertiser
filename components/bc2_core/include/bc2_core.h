/**
 * @file bc2_core.h
 * @brief Session state machine of the advertiser: command handling, set table, keep-alive
 *        watchdog, button handling and LED state derivation.
 *
 * Pure C11. All hardware access goes through the small ops structs below, which the unit tests
 * replace with fakes. The core is NOT thread safe: the caller serialises all calls with one mutex.
 */
#ifndef BC2_CORE_H
#define BC2_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bc2_gatt.h"
#include "bc2_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Capacity of the set table (the BLE adapter reports the real, possibly smaller, maximum). */
#define BC2_CORE_MAX_SETS 8u
/** How long the ERROR LED state lasts after a failure or a parse error burst. */
#define BC2_ERROR_DISPLAY_MS 2000u
/** How long the BUTTON_STOP LED state lasts. */
#define BC2_BUTTON_DISPLAY_MS 300u
/** How long the white activity flash after a SET_UPDATE lasts. */
#define BC2_ACTIVITY_FLASH_MS 60u

/** Parameters of one advertising set as handed to the BLE adapter. */
typedef struct {
    uint8_t flags;            /**< SET_START flag byte (AD flags bit, PDU type) */
    uint16_t interval;        /**< 0.625 ms units (min = max) */
    int8_t tx_power;          /**< dBm or BC2_TX_POWER_MAX */
    uint16_t mfr_id;          /**< manufacturer id (company identifier) */
    uint8_t data_len;         /**< raw data bytes */
    const uint8_t *data;      /**< raw data bytes (manufacturer specific payload) */
} bc2_set_params_t;

/** BLE adapter operations (implemented by bc2_ble on the device, by a fake in the tests). */
typedef struct {
    void *ctx;                                                        /**< passed to every call */
    bool (*ready)(void *ctx);                                         /**< host synced, sets can be started */
    uint8_t (*max_sets)(void *ctx);                                   /**< number of usable sets */
    void (*device_id)(void *ctx, uint8_t out[6]);                     /**< public MAC */
    /** Start a set. On BC2_ERR_BLE_ERROR @p rc carries the NimBLE return code. */
    bc2_status_t (*start)(void *ctx, uint8_t set_id, const bc2_set_params_t *p, int8_t *applied_tx,
                          uint16_t *rc);
    /** Replace the data of a running set. */
    bc2_status_t (*update)(void *ctx, uint8_t set_id, uint16_t mfr_id, const uint8_t *data,
                           uint8_t len, uint16_t *rc);
    /** Stop a set; returns false if the controller may still be advertising. */
    bool (*stop)(void *ctx, uint8_t set_id);
    /** Fetch one asynchronous set failure (restart after connection failed etc.). */
    bool (*take_failure)(void *ctx, uint8_t *set_id, uint8_t *reason);
} bc2_ble_ops_t;

/** I/O operations (USB write, time, emergency reset). */
typedef struct {
    void *ctx;                                                        /**< passed to every call */
    void (*write)(void *ctx, const uint8_t *data, size_t len);        /**< responses / state events: best effort */
    /**
     * Scan reports and notifications: may be refused when the TX queue is under pressure (returns
     * false, the core counts the drop). NULL = same as write() and always accepted.
     */
    bool (*write_droppable)(void *ctx, const uint8_t *data, size_t len);
    uint32_t (*now_us)(void *ctx);                                    /**< free running microsecond clock */
    void (*hard_reset)(void *ctx);                                    /**< reset the chip (guaranteed silence) */
} bc2_io_ops_t;

/** LED states, ordered by display priority (highest first) as documented in README. */
typedef enum {
    BC2_LED_ERROR,
    BC2_LED_BUTTON_STOP,
    BC2_LED_TIMEOUT,
    BC2_LED_ADVERTISING,
    BC2_LED_CONNECTED,
    BC2_LED_NO_HOST,
    BC2_LED_BOOT
} bc2_led_state_t;

/** Bookkeeping of one set slot. */
typedef struct {
    bool running;
    uint8_t flags;
    uint16_t interval;
    int8_t tx_power;                           /**< applied value reported by the BLE adapter */
    uint16_t mfr_id;
    uint8_t data_len;
    uint8_t data[BC2_MAX_DATA_NO_FLAGS];
} bc2_slot_t;

/** Core state. Treat as opaque; allocate it and call bc2_core_init(). */
typedef struct {
    bc2_ble_ops_t ble;
    bc2_io_ops_t io;
    bc2_framer_t framer;
    bc2_gatt_t gatt;               /**< scanner / GATT central logic (protocol 1.1) */
    bc2_slot_t slots[BC2_CORE_MAX_SETS];
    bool session;                  /**< a HELLO was accepted and no timeout happened since */
    bool timed_out;                /**< keep-alive timeout happened, cleared by HELLO */
    uint16_t keepalive_ms;         /**< effective keep-alive timeout */
    uint32_t last_rx_ms;           /**< time of the last valid frame */
    uint32_t error_until_ms;       /**< ERROR LED state active until this time (if error_active) */
    bool error_active;
    uint32_t button_until_ms;
    bool button_active;
    uint32_t activity_until_ms;
    bool activity_active;
    uint16_t seen_parse_errors;    /**< parse error counter value already reported to the LED */
} bc2_core_t;

/**
 * Initialise the core with its hardware operations. No session, no sets, no links.
 * @param gatt GATT adapter operations, or NULL for a build without GATT (requests answer NOT_SUPPORTED)
 */
void bc2_core_init(bc2_core_t *core, const bc2_ble_ops_t *ble, const bc2_gatt_ops_t *gatt, const bc2_io_ops_t *io);

/**
 * Feed received USB bytes; complete frames are handled and answered immediately.
 * @param now_ms millisecond clock since boot
 */
void bc2_core_on_bytes(bc2_core_t *core, const uint8_t *data, size_t len, uint32_t now_ms);

/**
 * Periodic housekeeping (call every <= 20 ms, and whenever the GATT adapter signals an event):
 * adapter events, GATT deadlines, keep-alive timeout, asynchronous set failures.
 */
void bc2_core_tick(bc2_core_t *core, uint32_t now_ms);

/** Button short press: stop all sets, links and the scan immediately, keep the session, emit EVT_BUTTON. */
void bc2_core_on_button(bc2_core_t *core, uint32_t now_ms);

/** LED state to display now. */
bc2_led_state_t bc2_core_led_state(const bc2_core_t *core, uint32_t now_ms);

/** True while a scan runs or a GATT link / connect attempt exists (shown like advertising). */
bool bc2_core_gatt_active(const bc2_core_t *core);

/** True during the short white flash after a SET_UPDATE (visible activity while ADVERTISING). */
bool bc2_core_activity_flash(const bc2_core_t *core, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BC2_CORE_H */
