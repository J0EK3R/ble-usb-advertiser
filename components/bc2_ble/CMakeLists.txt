# ESP32-only adapter: implements bc2_ble_ops_t with the NimBLE extended advertising API.
idf_component_register(SRCS "bc2_ble.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bc2_core
                       PRIV_REQUIRES bt esp_hw_support)
