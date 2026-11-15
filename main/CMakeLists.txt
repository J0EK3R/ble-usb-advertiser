# Application component: wires the other components together.
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES bc2_core bc2_ble bc2_usb bc2_led bc2_board
                       PRIV_REQUIRES esp_timer esp_system esp_driver_gpio)
