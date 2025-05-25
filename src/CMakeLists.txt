# Add blink executable
add_executable(blink
    blink.c
)

# Add main ShoulderShield application executable
add_executable(shoulder_shield
    main.c
    bno085.c
)

# Add BNO085 I2C example executable  
add_executable(bno085_example
    bno085_example.c
    bno085.c
)

# Add BNO085 UART-RVC example executable
add_executable(bno085_rvc_example
    bno085_rvc_example.c
    bno085_rvc.c
)

# Add BNO085 SPI example executable
add_executable(bno085_spi_example
    bno085_spi_example.c
    bno085_spi.c
)

# Enable USB and UART output for all applications
pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 1)

pico_enable_stdio_usb(shoulder_shield 1)
pico_enable_stdio_uart(shoulder_shield 1)

pico_enable_stdio_usb(bno085_example 1)
pico_enable_stdio_uart(bno085_example 1)

pico_enable_stdio_usb(bno085_rvc_example 1)
pico_enable_stdio_uart(bno085_rvc_example 1)

pico_enable_stdio_usb(bno085_spi_example 1)
pico_enable_stdio_uart(bno085_spi_example 1)

# Link libraries to blink
target_link_libraries(blink 
    pico_stdlib
)

# Link libraries to shoulder_shield
target_link_libraries(shoulder_shield
    pico_stdlib
    hardware_i2c
)

# Link libraries to bno085_example (I2C version)
target_link_libraries(bno085_example
    pico_stdlib
    hardware_i2c
)

# Link libraries to bno085_rvc_example (UART version)
target_link_libraries(bno085_rvc_example
    pico_stdlib
    hardware_uart
)

# Link libraries to bno085_spi_example (SPI version)
target_link_libraries(bno085_spi_example
    pico_stdlib
    hardware_spi
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(blink)
pico_add_extra_outputs(shoulder_shield)
pico_add_extra_outputs(bno085_example)
pico_add_extra_outputs(bno085_rvc_example)
pico_add_extra_outputs(bno085_spi_example)