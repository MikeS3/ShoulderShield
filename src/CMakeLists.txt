cmake_minimum_required(VERSION 3.13)

# Instruct CMake where the Pico SDK is located
include(pico_sdk_import.cmake)
project(bno08x_multi_project C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# ------------------------------
# Example: Multi-IMU BNO08x Demo with Multi-Instance Support
# ------------------------------
add_executable(Simple_test_3Imu
    3IMUmain.c
    Pico_BNO08x_multi.c
    sh2_multi.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

# Enable USB and UART output for this target
pico_enable_stdio_usb(Simple_test_3Imu 1)
pico_enable_stdio_uart(Simple_test_3Imu 1)

# Include directories (SDK and project)
target_include_directories(Simple_test_3Imu PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Link against Pico libraries (including I2C support)
target_link_libraries(Simple_test_3Imu
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
)

# Generate binary, hex, and uf2 files
pico_add_extra_outputs(Simple_test_3Imu)

# Export compile commands for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)