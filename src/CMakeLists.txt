# Add blink executable (keep existing)
add_executable(blink 
    blink.c
)

# Enable USB and UART output
pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 1)

# Link libraries
target_link_libraries(blink 
    pico_stdlib
)
if (PICO_CYW43_SUPPORTED)
    target_link_libraries(blink pico_cyw43_arch_none)
endif()

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(blink)

# Add BNO08x IMU example executable with your 3 IMU test main
add_executable(Simple_test_3Imu
    Simple_test_3Imu.c
    Pico_BNO08x_multi.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

# Enable USB and UART output for your example
pico_enable_stdio_usb(Simple_test_3Imu 1)
pico_enable_stdio_uart(Simple_test_3Imu 1)

# Include directories for BNO08x (assuming headers in current dir)
target_include_directories(Simple_test_3Imu PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link required hardware libraries and pico stdlib
target_link_libraries(Simple_test_3Imu
    pico_stdlib
    hardware_spi
    hardware_gpio
)

pico_add_extra_outputs(Simple_test_3Imu)
