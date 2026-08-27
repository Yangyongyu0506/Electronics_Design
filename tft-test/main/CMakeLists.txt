idf_component_register(SRCS "tft-test.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES
                        esp_driver_spi
                        esp_lcd)
