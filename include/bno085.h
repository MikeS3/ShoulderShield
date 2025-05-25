/**
 * BNO085 Sensor Interface for Raspberry Pi Pico W
 * Header file with definitions and function declarations
 */

#ifndef BNO085_H
#define BNO085_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "hardware/i2c.h"

// BNO085 I2C address options (depends on SA0 pin)
#define BNO085_I2C_ADDR_SA0_0 0x4A
#define BNO085_I2C_ADDR_SA0_1 0x4B

// Feature report IDs
#define BNO085_REPORT_ACCELEROMETER           0x01
#define BNO085_REPORT_GYROSCOPE               0x02
#define BNO085_REPORT_MAGNETIC_FIELD          0x03
#define BNO085_REPORT_LINEAR_ACCELERATION     0x04
#define BNO085_REPORT_ROTATION_VECTOR         0x05
#define BNO085_REPORT_GRAVITY                 0x06
#define BNO085_REPORT_GAME_ROTATION_VECTOR    0x08
#define BNO085_REPORT_GEOMAGNETIC_ROTATION    0x09
#define BNO085_REPORT_STEP_COUNTER            0x11
#define BNO085_REPORT_STABILITY_CLASSIFIER    0x13
#define BNO085_REPORT_GYRO_INTEGRATED_RV      0x2A

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
} bno085_quaternion_t;

/**
 * Initialize the BNO085 sensor
 * 
 * @param i2c_inst      I2C instance to use (i2c0 or i2c1)
 * @param i2c_sda_pin   GPIO pin number for SDA
 * @param i2c_scl_pin   GPIO pin number for SCL
 * @param intr_pin      GPIO pin number connected to BNO085 interrupt
 * @param address       I2C address of the BNO085 (usually 0x4A or 0x4B)
 * @return              true if initialization successful, false otherwise
 */
bool bno085_init(i2c_inst_t* i2c_inst, uint i2c_sda_pin, uint i2c_scl_pin, 
                 uint intr_pin, uint8_t address);

/**
 * Enable rotation vector (quaternion) reports from the BNO085
 * 
 * @param interval_us   Desired report interval in microseconds
 * @return              true if successful, false otherwise
 */
bool bno085_enable_rotation_vector(uint32_t interval_us);

/**
 * Read quaternion data from the BNO085
 * 
 * @param quat          Pointer to structure to store quaternion data
 * @return              true if successful, false otherwise
 */
bool bno085_read_quaternion(bno085_quaternion_t* quat);

#endif // BNO085_H