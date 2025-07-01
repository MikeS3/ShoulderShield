cmake_minimum_required(VERSION 3.13)

# Instruct CMake where the Pico SDK is located
include(pico_sdk_import.cmake)
project(bno08x_multi_fixed C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# ------------------------------
# Fixed Multi-IMU BNO08x Demo
# ------------------------------
add_executable(debug_test
    # Main application
    3IMUmainDebug.c
    
    # Fixed BNO08x driver
    Pico_BNO08x_debug.c
    
    # Fixed SH2 library
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

# Enable USB and UART output
pico_enable_stdio_usb(debug_test 1)
pico_enable_stdio_uart(debug_test 1)

# Include directories
target_include_directories(debug_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Link against Pico libraries
target_link_libraries(debug_test
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# Generate binary, hex, and uf2 files
pico_add_extra_outputs(debug_test)

# Export compile commands for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------
# Optional: Build original for comparison
# ------------------------------
add_executable(original_test
    3IMUmain.c
    Pico_BNO08x_multi.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

pico_enable_stdio_usb(original_test 1)
pico_enable_stdio_uart(original_test 1)

target_include_directories(original_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(original_test
    pico_stdlib
    hardware_spi
    hardware_gpio
)

pico_add_extra_outputs(original_test)