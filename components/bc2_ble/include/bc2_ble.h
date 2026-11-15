/**
 * @file bc2_ble.h
 * @brief NimBLE adapter: runs up to BC2ADV_MAX_SETS legacy advertising sets, one per NimBLE
 *        extended-advertising instance (protocol set_id n == instance n).
 */
#ifndef BC2_BLE_H
#define BC2_BLE_H

#include "bc2_core.h"
#include "esp_err.h"

/** Initialise the controller and the NimBLE host and start the host task. Returns immediately;
 *  bc2_ble_ops_t.ready() turns true when the host has synced with the controller. */
esp_err_t bc2_ble_init(void);

/** Operations table for bc2_core_init(). Valid after bc2_ble_init(). */
const bc2_ble_ops_t *bc2_ble_ops(void);

#endif /* BC2_BLE_H */
