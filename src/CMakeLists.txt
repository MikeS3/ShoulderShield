# Add blink executable (keep existing)

# Enable USB and UART output
# pico_enable_stdio_usb(blink 1)
# pico_enable_stdio_uart(blink 1)

# Link libraries
#target_link_libraries(blink 
#    pico_stdlib
#)
#if (PICO_CYW43_SUPPORTED)
#    target_link_libraries(blink pico_cyw43_arch_none)
#endif()

# Create map/bin/hex/uf2 files
# pico_add_extra_outputs(blink)

# Add BNO08x IMU example executable
set(SOURCE_FILES main.c)
aux_source_directory(sh2 SOURCE_FILES)
aux_source_directory(bno085 SOURCE_FILES)

add_executable(bno08x_example
	${SOURCE_FILES}
)

# Enable USB and UART output for this target
pico_enable_stdio_usb(Simple_test_3Imu 1)
pico_enable_stdio_uart(Simple_test_3Imu 1)

# Include directories (SDK and project)
target_include_directories(Simple_test_3Imu PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against Pico libraries
target_link_libraries(Simple_test_3Imu
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# Create map/bin/hex/uf2 files for BNO08x example
pico_add_extra_outputs(bno08x_example)
