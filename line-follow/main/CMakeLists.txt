idf_component_register(SRCS "line-follow.c" "tft_display.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_spi esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_timer)
