file(GLOB_RECURSE SRC "*.cpp")

idf_component_register(SRCS ${SRC}
                       INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23)