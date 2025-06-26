/*!
 *  @file Pico_BNO08x.h
 *
 *  Raspberry Pi Pico Driver for the BNO08x 9-DOF Orientation IMU Fusion
 *
 *  Ported from Adafruit BNO08x library to work with Raspberry Pi Pico SDK
 *  Supports multiple IMU instances via SPI
 *
 *  @section dependencies Dependencies
 *  This library depends on the Raspberry Pi Pico SDK and SH2 library with .cookie support
 *
 *  @section author Author
 *  Ported for Raspberry Pi Pico
 *
 *  @section license License
 *  BSD (see license.txt)
 */

#ifndef PICO_BNO08X_H
#define PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9 ///< The number of current options for the activity classifier

/*!
 *    @brief  Struct that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout on Raspberry Pi Pico
 *            Supports multiple instances for multi-IMU setups
 */
typedef struct {
    // SPI Hardware interface
    spi_inst_t *spi_port;
    
    // SPI Pin configurations
    uint8_t cs_pin;
    uint8_t int_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    
    // Reset pin
    int8_t reset_pin;
    
    // SPI settings
    uint32_t spi_speed;
    
    // SH2 HAL (must have .cookie field for multi-IMU support)
    sh2_Hal_t hal;
    
    // Product IDs
    sh2_ProductIds_t prodIds;
    
    // Reset flag
    bool reset_occurred;
    
    // Instance identifier (useful for debugging multi-IMU setups)
    uint8_t instance_id;
    
} Pico_BNO08x_t;

// Function prototypes

/**
 * @brief Initialize BNO08x instance structure
 * @param bno Pointer to BNO08x instance
 * @param reset_pin GPIO pin for reset (-1 if not used)
 * @param instance_id Unique ID for this IMU instance
 * @return true if successful, false otherwise
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, int8_t reset_pin, uint8_t instance_id);

/**
 * @brief Initialize BNO08x with SPI interface
 * @param bno Pointer to BNO08x instance
 * @param spi_port SPI port (spi0 or spi1)
 * @param miso_pin MISO GPIO pin
 * @param mosi_pin MOSI GPIO pin
 * @param sck_pin SCK GPIO pin
 * @param cs_pin Chip select GPIO pin
 * @param int_pin Interrupt GPIO pin
 * @param spi_speed SPI clock speed in Hz
 * @return true if successful, false otherwise
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                          uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed);

/**
 * @brief Perform hardware reset of the IMU
 * @param bno Pointer to BNO08x instance
 */
void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno);

/**
 * @brief Check if IMU was reset
 * @param bno Pointer to BNO08x instance
 * @return true if reset occurred, false otherwise
 */
bool pico_bno08x_was_reset(Pico_BNO08x_t *bno);

/**
 * @brief Enable a sensor report
 * @param bno Pointer to BNO08x instance
 * @param sensor_id Sensor ID to enable
 * @param interval_us Report interval in microseconds
 * @return true if successful, false otherwise
 */
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us);

/**
 * @brief Get sensor event/reading
 * @param bno Pointer to BNO08x instance
 * @param value Pointer to store sensor value
 * @return true if event received, false otherwise
 */
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);

/**
 * @brief Service the IMU (call regularly in main loop)
 * @param bno Pointer to BNO08x instance
 */
void pico_bno08x_service(Pico_BNO08x_t *bno);

// multi‑IMU “active” selector
void pico_bno08x_set_active(Pico_BNO08x_t *bno);

#endif
