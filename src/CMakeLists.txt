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

# Enable USB and UART output for BNO08x example
pico_enable_stdio_usb(bno08x_example 1)
pico_enable_stdio_uart(bno08x_example 1)

# Include directories for BNO08x
target_include_directories(bno08x_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link libraries for BNO08x example
target_link_libraries(bno08x_example 
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_uart
    hardware_gpio
)

# Create map/bin/hex/uf2 files for BNO08x example
pico_add_extra_outputs(bno08x_example)
