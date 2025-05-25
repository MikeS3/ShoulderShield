/**
 * BNO085 SPI Interface for Raspberry Pi Pico W
 * Header file for SPI-based communication with BNO085
 */

#ifndef BNO085_SPI_H
#define BNO085_SPI_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "hardware/spi.h"

// BNO085 Feature report IDs
#define BNO085_REPORT_ACCELEROMETER           0x01
#define BNO085_REPORT_GYROSCOPE               0x02
#define BNO085_REPORT_MAGNETIC_FIELD          0x03
#define BNO085_REPORT_LINEAR_ACCELERATION     0x04
#define BNO085_REPORT_ROTATION_VECTOR         0x05
#define BNO085_REPORT_GRAVITY                 0x06
#define BNO085_REPORT_GAME_ROTATION_VECTOR    0x08
#define BNO085_REPORT_GEOMAGNETIC_ROTATION    0x09

// Accuracy status values
#define BNO085_ACCURACY_UNRELIABLE 0
#define BNO085_ACCURACY_LOW        1
#define BNO085_ACCURACY_MEDIUM     2
#define BNO085_ACCURACY_HIGH       3

// Quaternion data structure
typedef struct {
    float real;             // Real (scalar) component (w)
    float i;                // i component (x)
    float j;                // j component (y)
    float k;                // k component (z)
    float accuracy_rad;     // Estimated accuracy in radians
    uint8_t accuracy_status; // Accuracy status (0-3)
} bno085_spi_quaternion_t;

/**
 * @brief Initialize the BNO085 sensor with SPI communication
 * 
 * @param spi_inst      SPI instance to use (spi0 or spi1)
 * @param miso_pin      GPIO pin number for MISO (Master In, Slave Out)
 * @param mosi_pin      GPIO pin number for MOSI (Master Out, Slave In)
 * @param sck_pin       GPIO pin number for SCK (Serial Clock)
 * @param cs_pin        GPIO pin number for CS (Chip Select)
 * @param intr_pin      GPIO pin number connected to BNO085 interrupt
 * @param wake_pin      GPIO pin number connected to BNO085 wake (PS0)
 * @return              true if initialization successful, false otherwise
 */
bool bno085_spi_init(spi_inst_t* spi_inst, uint miso_pin, uint mosi_pin, 
                     uint sck_pin, uint cs_pin, uint intr_pin, uint wake_pin);

/**
 * @brief Enable rotation vector (quaternion) reports from the BNO085
 * 
 * @param interval_us   Desired report interval in microseconds
 * @return              true if successful, false otherwise
 */
bool bno085_spi_enable_rotation_vector(uint32_t interval_us);

/**
 * @brief Read quaternion data from the BNO085
 * 
 * @param quat          Pointer to structure to store quaternion data
 * @return              true if successful, false otherwise
 */
bool bno085_spi_read_quaternion(bno085_spi_quaternion_t* quat);

#endif // BNO085_SPI_H