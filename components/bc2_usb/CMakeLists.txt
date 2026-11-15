# USB Serial/JTAG transport (virtual COM port without driver on Windows 10/11).
idf_component_register(SRCS "bc2_usb.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_usb_serial_jtag)
