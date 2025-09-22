cmake_minimum_required(VERSION 3.13)
# Instruct CMake where the Pico SDK is located
include(pico_sdk_import.cmake)
project(bno08x_multi_fixed C CXX ASM)
# Initialize the Pico SDK
pico_sdk_init()

add_executable(shoulder_shield
    main.c
    Pico_BNO08x_multi.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)
pico_enable_stdio_usb(shoulder_shield 1)
pico_enable_stdio_uart(shoulder_shield 1)
target_include_directories(shoulder_shield PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(shoulder_shield
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_add_extra_outputs(shoulder_shield)