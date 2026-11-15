# RGB LED pattern output (WS2812 via the led_strip managed component).
idf_component_register(SRCS "bc2_led.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bc2_core bc2_board
                       PRIV_REQUIRES led_strip)
