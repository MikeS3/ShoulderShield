/*!
 *  @file bno085_rvc.c
 *
 *  BNO085 UART-RVC Mode Library for Raspberry Pi Pico
 *  Converted from Adafruit's Arduino library
 */

#include "bno085_rvc.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// Global variables
static uart_inst_t *uart_instance = NULL;

/**
 * @brief Initialize BNO085 in UART-RVC mode
 */
bool bno085_rvc_init(uart_inst_t *uart_inst, uint tx_pin, uint rx_pin, uint baudrate) {
    uart_instance = uart_inst;
    
    // Initialize UART
    uart_init(uart_instance, baudrate);
    
    // Set up GPIO pins for UART
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // Set UART format: 8 data bits, 1 stop bit, no parity
    uart_set_format(uart_instance, 8, 1, UART_PARITY_NONE);
    
    // Enable UART FIFO
    uart_set_fifo_enabled(uart_instance, true);
    
    printf("BNO085 RVC: UART initialized at %u baud\n", baudrate);
    printf("BNO085 RVC: Waiting for data packets...\n");
    
    return true;
}

/**
 * @brief Check if data is available to read
 */
bool bno085_rvc_available(void) {
    if (!uart_instance) {
        return false;
    }
    return uart_is_readable(uart_instance);
}

/**
 * @brief Read a single byte from UART (non-blocking)
 */
static bool read_byte(uint8_t *byte) {
    if (!uart_is_readable(uart_instance)) {
        return false;
    }
    *byte = uart_getc(uart_instance);
    return true;
}

/**
 * @brief Peek at the next byte without consuming it
 */
static bool peek_byte(uint8_t *byte) {
    // Unfortunately, Pico SDK doesn't have a direct peek function
    // We'll implement this by checking if data is available
    if (!uart_is_readable(uart_instance)) {
        return false;
    }
    
    // For now, we'll just indicate data is available
    // The actual peek will be done by reading and handling in the main read function
    return true;
}

/**
 * @brief Read multiple bytes with timeout
 */
static bool read_bytes_timeout(uint8_t *buffer, size_t length, uint32_t timeout_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    size_t bytes_read = 0;
    
    while (bytes_read < length) {
        if (to_ms_since_boot(get_absolute_time()) - start_time > timeout_ms) {
            return false; // Timeout
        }
        
        if (uart_is_readable(uart_instance)) {
            buffer[bytes_read] = uart_getc(uart_instance);
            bytes_read++;
        } else {
            sleep_us(100); // Small delay to prevent busy waiting
        }
    }
    
    return true;
}

/**
 * @brief Read BNO085 RVC data packet
 */
bool bno085_rvc_read(bno085_rvc_data_t *data) {
    if (!data || !uart_instance) {
        return false;
    }
    
    // Check if any data is available
    if (!uart_is_readable(uart_instance)) {
        return false;
    }
    
    // Look for the first header byte (0xAA)
    uint8_t byte;
    if (!read_byte(&byte) || byte != BNO085_RVC_HEADER_1) {
        return false; // First byte is not 0xAA or no data available
    }
    
    // Check for the second header byte (0xAA)
    if (!read_byte(&byte) || byte != BNO085_RVC_HEADER_2) {
        return false; // Second byte is not 0xAA
    }
    
    // We found the header, now read the remaining 17 bytes
    uint8_t buffer[17];
    if (!read_bytes_timeout(buffer, 17, 100)) { // 100ms timeout
        printf("BNO085 RVC: Timeout reading packet data\n");
        return false;
    }
    
    // Calculate checksum on the first 16 bytes
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) {
        sum += buffer[i];
    }
    
    // Verify checksum
    if (sum != buffer[16]) {
        printf("BNO085 RVC: Checksum mismatch (calculated: %02X, received: %02X)\n", sum, buffer[16]);
        return false;
    }
    
    // Parse the data - convert little-endian 16-bit values
    int16_t values[6];
    for (uint8_t i = 0; i < 6; i++) {
        values[i] = (int16_t)(buffer[1 + (i * 2)] | (buffer[1 + (i * 2) + 1] << 8));
    }
    
    // Convert to floating point with proper scaling
    data->yaw = (float)values[0] * DEGREE_SCALE;
    data->pitch = (float)values[1] * DEGREE_SCALE;
    data->roll = (float)values[2] * DEGREE_SCALE;
    
    data->x_accel = (float)values[3] * MILLI_G_TO_MS2;
    data->y_accel = (float)values[4] * MILLI_G_TO_MS2;
    data->z_accel = (float)values[5] * MILLI_G_TO_MS2;
    
    return true;
}