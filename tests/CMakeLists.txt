cmake_minimum_required(VERSION 3.20)
project(EZDCFirmwareTests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(tests
    src/fixed_test.cpp
    src/rational_test.cpp
    src/SPSCRingbuffer_test.cpp
    src/MPSCRingbuffer_test.cpp
    src/PiecewiseLinearFunction_test.cpp
    src/CRC_test.cpp
    src/Angle_test.cpp
)

target_include_directories(tests PRIVATE
    ${PROJECT_SOURCE_DIR}/../main
)

target_link_libraries(tests
    GTest::gtest_main
)

target_compile_options(tests PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-character-conversion
    -g
    -O0
)
target_compile_options(gtest PRIVATE -Wno-character-conversion)
target_compile_options(gtest_main PRIVATE -Wno-character-conversion)

include(GoogleTest)
gtest_discover_tests(tests)
