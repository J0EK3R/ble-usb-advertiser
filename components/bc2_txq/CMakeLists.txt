# Pure C frame queue (no ESP headers) between the producers (core, BLE) and the USB TX task.
idf_component_register(SRCS "bc2_txq.c"
                       INCLUDE_DIRS "include")
