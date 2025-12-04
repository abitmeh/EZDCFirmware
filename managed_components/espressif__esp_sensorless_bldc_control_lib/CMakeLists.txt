idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS ".")

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bldc_snls_lib.c")
    target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_IDF_TARGET}/libesp_sensorless_bldc_control_lib.a")
endif()