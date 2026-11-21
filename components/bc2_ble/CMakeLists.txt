# ESP32-only adapters: bc2_ble implements bc2_ble_ops_t (NimBLE extended advertising),
# bc2_gatt_nimble implements bc2_gatt_ops_t (scanner + GATT central, only if BC2ADV_ENABLE_GATT).
set(srcs "bc2_ble.c")
if(CONFIG_BC2ADV_ENABLE_GATT)
    list(APPEND srcs "bc2_gatt_nimble.c")
endif()
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES bc2_core
                       PRIV_REQUIRES bt esp_hw_support esp_timer esp_system bc2_scanfilter)
