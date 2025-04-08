# Add blink executable
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
