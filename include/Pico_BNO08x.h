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
#include "pico/stdlib.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief BNO08x device state and configuration for a single instance
 *
 * Each BNO08x instance holds its own SPI config, SH2 HAL, interrupt, and
 * pending sensor data, allowing true multi-IMU use.
 */
typedef struct {
      // Hardware SPI config
    spi_inst_t *spi_port;     ///< SPI port (e.g., spi0 or spi1)
    uint8_t cs_pin;           ///< Chip select pin (active low)
    uint8_t int_pin;          ///< Interrupt pin (active low)
    uint8_t reset_pin;        ///< Reset pin (active low)
    uint32_t spi_frequency;   ///< SPI clock speed in Hz
    int instance_id;          ///< Optional user-specified instance ID
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    
    // SH2 HAL layer (each instance gets its own)
    sh2_Hal_t hal;

    // Sensor value buffer
    sh2_SensorValue_t sensor_value;      ///< Most recent sensor data
    sh2_SensorValue_t *pending_value;    ///< Pointer to data ready to be read

    // State flags
    bool initialized;
    bool spi_begun;
    bool has_reset;

    // Timing
    uint32_t last_reset_time_us;
    uint32_t last_service_time_us;
    
} Pico_BNO08x_t;

// Function prototypes
/**
 * @brief Initialize BNO08x instance (does not start SPI)
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, int reset_pin, int instance_id);

/**
 * @brief Start SPI communication and register SH2 callbacks
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                           uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                           uint8_t cs_pin, uint8_t int_pin, uint32_t frequency);

// Service function (replaces set_active pattern)
void pico_bno08x_service(Pico_BNO08x_t *bno);
void hardware_reset(Pico_BNO08x_t *bno);

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
