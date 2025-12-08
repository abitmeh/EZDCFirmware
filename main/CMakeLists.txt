file(GLOB_RECURSE SRC "*.cpp")

idf_component_register(SRCS ${SRC}
                       INCLUDE_DIRS "."
                       REQUIRES esp_pp_idf esp_event esp_driver_spi esp_timer)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23)