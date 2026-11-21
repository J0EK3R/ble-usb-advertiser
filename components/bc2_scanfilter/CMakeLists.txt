# Pure C scan report filter + address type cache (no ESP headers), used by the NimBLE scan adapter.
idf_component_register(SRCS "bc2_scanfilter.c"
                       INCLUDE_DIRS "include")
