idf_component_register(SRCS "cooperative_jpeg.c" "cam-ball-track.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES
                        usb
                        esp_ringbuf
                        esp_timer
                        esp_jpeg
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_pcnt)
