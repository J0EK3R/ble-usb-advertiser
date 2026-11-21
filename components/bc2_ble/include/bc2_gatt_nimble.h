/**
 * @file bc2_gatt_nimble.h
 * @brief NimBLE adapter for the GATT central / scanner logic (implements bc2_gatt_ops_t).
 *
 * ESP32 only. Scan, connect, MTU exchange, discovery, read/write/subscribe and notifications run in
 * the NimBLE host task; results are queued as bc2_gatt_event_t and polled by bc2_core under its
 * mutex (see bc2_gatt.h). The adapter never calls into bc2_core.
 */
#ifndef BC2_GATT_NIMBLE_H
#define BC2_GATT_NIMBLE_H

#include "bc2_gatt.h"
#include "esp_err.h"

/**
 * Create the event queue and configure the ATT MTU. Call after bc2_ble_init() (NimBLE is running).
 * @param wake called (from the NimBLE host task) whenever an event was queued, so the core task can
 *             process it at once instead of waiting for its 20 ms tick; may be NULL
 */
esp_err_t bc2_gatt_nimble_init(void (*wake)(void));

/** Operations table for bc2_core_init(). Valid after bc2_gatt_nimble_init(). */
const bc2_gatt_ops_t *bc2_gatt_nimble_ops(void);

#endif /* BC2_GATT_NIMBLE_H */
