# Pure C session/state-machine component (no ESP headers). Talks to the hardware only through
# the ops structs declared in bc2_core.h.
idf_component_register(SRCS "bc2_core.c" "bc2_gatt.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bc2_protocol)
