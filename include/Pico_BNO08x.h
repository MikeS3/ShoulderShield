#ifndef PICO_BNO08X_H
#define PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO08x_I2CADDR_DEFAULT 0x4A

// Interface types
typedef enum {
    INTERFACE_NONE = 0,
    INTERFACE_SPI,
    INTERFACE_I2C,
    INTERFACE_UART
} interface_type_t;

/**
 * @brief BNO08x device state and configuration for a single instance
 *
 * Each BNO08x instance holds its own interface config, SH2 HAL, interrupt, and
 * pending sensor data, allowing true multi-IMU use.
 */
typedef struct {
    // Hardware SPI config
    spi_inst_t *spi_port;     ///< SPI port (e.g., spi0 or spi1)
    uint8_t cs_pin;           ///< Chip select pin (active low)
    uint8_t int_pin;          ///< Interrupt pin (active low)
    uint8_t reset_pin;        ///< Reset pin (active low)
    uint32_t spi_frequency;   ///< SPI clock speed in Hz
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t sck_pin;
    
    // Hardware I2C config
    i2c_inst_t *i2c_port;     ///< I2C port (e.g., i2c0 or i2c1)
    uint8_t sda_pin;          ///< I2C SDA pin
    uint8_t scl_pin;          ///< I2C SCL pin
    uint8_t i2c_addr;         ///< I2C slave address
    uint32_t i2c_speed;       ///< I2C clock speed in Hz
    
    // Interface type
    interface_type_t interface_type;
    
    int instance_id;          ///< Optional user-specified instance ID
    
    // SH2 HAL layer (each instance gets its own)
    sh2_Hal_t hal;

    // Sensor value buffer
    sh2_SensorValue_t sensor_value;      ///< Most recent sensor data
    sh2_SensorValue_t *pending_value;    ///< Pointer to data ready to be read

    // State flags
    bool initialized;
    bool spi_begun;
    bool has_reset;
    bool reset_occurred;

    // Timing
    uint32_t last_reset_time_us;
    uint32_t last_service_time_us;
    
} Pico_BNO08x_t;

// Function prototypes
/**
 * @brief Initialize BNO08x instance (does not start communication)
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, int reset_pin, int instance_id);

/**
 * @brief Start SPI communication and register SH2 callbacks
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                           uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                           uint8_t cs_pin, uint8_t int_pin, uint32_t frequency);

/**
 * @brief Start I2C communication and register SH2 callbacks
 */
bool pico_bno08x_begin_i2c(Pico_BNO08x_t *bno, i2c_inst_t *i2c_port,
                           uint8_t sda_pin, uint8_t scl_pin,
                           uint8_t i2c_addr, uint32_t i2c_speed);

// Service function (replaces set_active pattern)
void pico_bno08x_service(Pico_BNO08x_t *bno);

// Sensor configuration
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us);
bool pico_bno08x_disable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id);

// Data retrieval
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);
bool pico_bno08x_data_available(Pico_BNO08x_t *bno);

// Utility functions
void pico_bno08x_reset(Pico_BNO08x_t *bno);
bool pico_bno08x_soft_reset(Pico_BNO08x_t *bno);

// HAL callback functions (instance-aware via cookie)
int pico_bno08x_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
int pico_bno08x_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
uint32_t pico_bno08x_hal_get_time_us(sh2_Hal_t *self);

// SH2 callback functions (instance-aware via cookie)
void pico_bno08x_sensor_event(void *cookie, sh2_SensorEvent_t *pEvent);
void pico_bno08x_sensor_value(void *cookie, sh2_SensorValue_t *pValue);

#ifdef __cplusplus
}
#endif

#endif // PICO_BNO08X_H