file(GLOB_RECURSE SRC "*.cpp")

idf_component_register(SRCS ${SRC}
                       INCLUDE_DIRS "."
                       REQUIRES esp_pp_idf esp_event esp_driver_spi esp_driver_usb_serial_jtag esp_mm esp_timer esp_ringbuf)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23)