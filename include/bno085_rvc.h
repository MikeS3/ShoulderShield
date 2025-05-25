/*!
 *  @file bno085_rvc.h
 *
 *  BNO085 UART-RVC Mode Library for Raspberry Pi Pico
 *  Converted from Adafruit's Arduino library
 *
 *  This implements the simple UART-RVC mode of the BNO085 sensor
 *  which automatically sends yaw, pitch, roll, and acceleration data
 *  at 100Hz without needing complex initialization.
 */

#ifndef BNO085_RVC_H
#define BNO085_RVC_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/uart.h"

// Conversion constants
#define MILLI_G_TO_MS2 0.0098067f  // Scalar to convert milli-gs to m/s^2
#define DEGREE_SCALE 0.01f         // To convert the degree values

// UART-RVC packet constants
#define BNO085_RVC_PACKET_SIZE 19
#define BNO085_RVC_HEADER_1 0xAA
#define BNO085_RVC_HEADER_2 0xAA

/**
 * @brief Structure to hold BNO085 RVC data
 */
typedef struct {
    float yaw;      // Yaw in degrees
    float pitch;    // Pitch in degrees  
    float roll;     // Roll in degrees
    float x_accel;  // X acceleration in m/s^2
    float y_accel;  // Y acceleration in m/s^2
    float z_accel;  // Z acceleration in m/s^2
} bno085_rvc_data_t;

/**
 * @brief Initialize BNO085 in UART-RVC mode
 * 
 * @param uart_inst UART instance (uart0 or uart1)
 * @param tx_pin GPIO pin for UART TX (connect to BNO085 RX)
 * @param rx_pin GPIO pin for UART RX (connect to BNO085 TX)
 * @param baudrate UART baud rate (should be 115200 for RVC mode)
 * @return true if successful, false otherwise
 */
bool bno085_rvc_init(uart_inst_t *uart_inst, uint tx_pin, uint rx_pin, uint baudrate);

/**
 * @brief Read BNO085 RVC data packet
 * 
 * @param data Pointer to structure to store the data
 * @return true if valid packet received, false otherwise
 */
bool bno085_rvc_read(bno085_rvc_data_t *data);

/**
 * @brief Check if data is available to read
 * 
 * @return true if data is available, false otherwise
 */
bool bno085_rvc_available(void);

#endif // BNO085_RVC_H