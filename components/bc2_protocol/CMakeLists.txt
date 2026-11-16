# Pure C protocol library (no ESP headers). Used by the firmware and by the host unit tests.
idf_component_register(SRCS "bc2_crc16.c" "bc2_framer.c" "bc2_codec.c"
                       INCLUDE_DIRS "include")
