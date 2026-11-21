# USB Serial/JTAG transport (virtual COM port without driver on Windows 10/11) with a non blocking TX queue.
idf_component_register(SRCS "bc2_usb.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bc2_txq
                       PRIV_REQUIRES esp_driver_usb_serial_jtag)
