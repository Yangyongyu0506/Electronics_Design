idf_component_register(SRCS "imu-test.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES
                        esp_driver_i2c
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_pcnt
                        esp_timer
                        esp_wifi
                        esp_event
                        esp_netif
                        nvs_flash
                        lwip)
