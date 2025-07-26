cmake_minimum_required(VERSION 3.13)
# Instruct CMake where the Pico SDK is located
include(pico_sdk_import.cmake)
project(bno08x_multi_fixed C CXX ASM)
# Initialize the Pico SDK
pico_sdk_init()

add_executable(opensim_data
    OpenSimData.c
    Pico_BNO08x_multi.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)
pico_enable_stdio_usb(opensim_data 1)
pico_enable_stdio_uart(opensim_data 1)
target_include_directories(opensim_data PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(opensim_data
    pico_stdlib
    hardware_spi
    hardware_gpio
)
pico_add_extra_outputs(opensim_data)

# ------------------------------
# I2C Multi‑IMU BNO085 Demo
# ------------------------------
add_executable(I2C_Test
    I2C3ImuMain.c       # your main loop
    sh2M.c
    Pico_BNO085_I2C.C
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

pico_enable_stdio_usb(I2C_Test 1)
pico_enable_stdio_uart(I2C_Test 1)

target_include_directories(I2C_Test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    
    )

target_link_libraries(I2C_Test
    pico_stdlib
    hardware_i2c         # <-- use I²C driver
    hardware_gpio        # for pull‑ups & reset pins
)

pico_add_extra_outputs(I2C_Test)

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