idf_component_register(SRCS "cam-line-follow-and-ball-track.c" "cooperative_jpeg.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES
                        usb
                        esp_ringbuf
                        esp_timer
                        esp_jpeg
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_pcnt)

# sdkconfig.defaults selects the external decoder.  An existing sdkconfig can
# still contain CONFIG_JD_USE_ROM=y until it is regenerated, in which case the
# managed esp_jpeg component deliberately does not compile tjpgd.c or export
# its header.  Build the exact external source locally for that transitional
# configuration so cooperative_jpeg.c is always buildable.  Once sdkconfig is
# regenerated with CONFIG_JD_USE_ROM=n, esp_jpeg owns tjpgd.c and this fallback
# disappears, preventing duplicate symbols.
set(tjpgd_dir "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp_jpeg/tjpgd")
target_include_directories(${COMPONENT_LIB} PRIVATE "${tjpgd_dir}")

if(CONFIG_JD_USE_ROM)
    target_sources(${COMPONENT_LIB} PRIVATE
        "${tjpgd_dir}/tjpgd.c"
        "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp_jpeg/jpeg_default_huffman_table.c")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        CONFIG_JD_SZBUF=1024
        CONFIG_JD_FORMAT=0
        CONFIG_JD_USE_SCALE=1
        CONFIG_JD_TBLCLIP=1
        CONFIG_JD_FASTDECODE=1
        CONFIG_JD_DEFAULT_HUFFMAN=1)
endif()
