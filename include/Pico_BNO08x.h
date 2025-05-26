/*!
 *  @file Pico_BNO08x.h
 *
 *  Raspberry Pi Pico Driver for the BNO08x 9-DOF Orientation IMU Fusion
 *
 *  Ported from Adafruit BNO08x library to work with Raspberry Pi Pico SDK
 *
 *  @section dependencies Dependencies
 *  This library depends on the Raspberry Pi Pico SDK
 *
 *  @section author Author
 *  Ported for Raspberry Pi Pico
 *
 *  @section license License
 *  BSD (see license.txt)
 */

#ifndef _PICO_BNO08X_H
#define _PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9 ///< The number of current options for the activity classifier

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout on Raspberry Pi Pico
 */
typedef struct {
    // Hardware interfaces
    i2c_inst_t *i2c_port;
    spi_inst_t *spi_port;
    uart_inst_t *uart_port;
    
    // Pin configurations
    int8_t reset_pin;
    int8_t int_pin;
    uint8_t cs_pin;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    uint8_t tx_pin;
    uint8_t rx_pin;
    
    // I2C settings
    uint8_t i2c_addr;
    uint32_t i2c_speed;
    
    // SPI settings
    uint32_t spi_speed;
    
    // UART settings
    uint32_t uart_baudrate;
    
    // Interface type
    enum {
        INTERFACE_NONE,
        INTERFACE_I2C,
        INTERFACE_SPI,
        INTERFACE_UART
    } interface_type;
    
    // SH2 HAL
    sh2_Hal_t hal;
    
    // Product IDs
    sh2_ProductIds_t prodIds;
    
    // Reset flag
    bool reset_occurred;
    
} Pico_BNO08x_t;

// Function prototypes
bool pico_bno08x_init(Pico_BNO08x_t *bno, int8_t reset_pin);

// I2C interface functions
bool pico_bno08x_begin_i2c(Pico_BNO08x_t *bno, i2c_inst_t *i2c_port, 
                           uint8_t sda_pin, uint8_t scl_pin,
                           uint8_t i2c_addr, uint32_t i2c_speed);

// SPI interface functions  
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                          uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed);

// UART interface functions
bool pico_bno08x_begin_uart(Pico_BNO08x_t *bno, uart_inst_t *uart_port,
                            uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate);

// Common functions
void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno);
bool pico_bno08x_was_reset(Pico_BNO08x_t *bno);
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us);
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);

// Utility functions
void pico_bno08x_service(Pico_BNO08x_t *bno);

#endif