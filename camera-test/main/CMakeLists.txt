idf_component_register(SRCS "camera-test.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES
                        nvs_flash
                        usb
                        esp_ringbuf
                        esp_timer
                        esp_wifi
                        esp_driver_gpio
                        esp_event
                        esp_netif
                        esp_http_server)
