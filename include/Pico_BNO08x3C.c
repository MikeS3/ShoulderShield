/*!
 *  @file Pico_BNO08x.h
 *
 *  Raspberry Pi Pico Driver for Multiple BNO08x 9-DOF Orientation IMU Fusion
 *
 *  Modified to support multiple IMU instances
 */

#ifndef PICO_BNO08X_H
#define PICO_BNO08X_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of IMUs supported
#define MAX_IMU_COUNT 3

// Default I2C address
#define BNO08x_I2CADDR_DEFAULT 0x4A

// Interface types
typedef enum {
    INTERFACE_NONE = 0,
    INTERFACE_I2C,
    INTERFACE_SPI,
    INTERFACE_UART
} interface_type_t;

// BNO08x structure for individual IMU
typedef struct {
    // Hardware interface
    interface_type_t interface_type;
    
    // SPI interface (focus on SPI only now)
    spi_inst_t *spi_port;
    uint8_t cs_pin;
    uint8_t int_pin;
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t sck_pin;
    uint32_t spi_speed;
    
    // Reset pin
    int8_t reset_pin;
    
    // Status flags
    bool reset_occurred;
    bool initialized;
    
    // SH2 HAL
    sh2_Hal_t hal;
    
    // Product IDs
    sh2_ProductIds_t prodIds;
    
    // IMU identifier
    uint8_t imu_id;
    
} Pico_BNO08x_t;

// Multi-IMU manager structure
typedef struct {
    Pico_BNO08x_t imus[MAX_IMU_COUNT];
    uint8_t active_imu_count;
    uint8_t current_imu_index;
} Multi_BNO08x_t;

// Function prototypes for individual IMU
bool pico_bno08x_init(Pico_BNO08x_t *bno, uint8_t imu_id, int8_t reset_pin);
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                          uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed);
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us);
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);
void pico_bno08x_service(Pico_BNO08x_t *bno);
bool pico_bno08x_was_reset(Pico_BNO08x_t *bno);
void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno);

// Function prototypes for multi-IMU management
bool multi_bno08x_init(Multi_BNO08x_t *multi_bno);
bool multi_bno08x_add_spi_imu(Multi_BNO08x_t *multi_bno, uint8_t imu_index,
                             spi_inst_t *spi_port, uint8_t miso_pin, uint8_t mosi_pin, 
                             uint8_t sck_pin, uint8_t cs_pin, uint8_t int_pin, 
                             int8_t reset_pin, uint32_t spi_speed);
bool multi_bno08x_enable_all_reports(Multi_BNO08x_t *multi_bno, sh2_SensorId_t sensor_id, uint32_t interval_us);
void multi_bno08x_service_all(Multi_BNO08x_t *multi_bno);
bool multi_bno08x_get_sensor_event(Multi_BNO08x_t *multi_bno, uint8_t imu_index, sh2_SensorValue_t *value);
uint8_t multi_bno08x_get_imu_count(Multi_BNO08x_t *multi_bno);
bool multi_bno08x_is_imu_initialized(Multi_BNO08x_t *multi_bno, uint8_t imu_index);

#ifdef __cplusplus
}
#endif

#endif // PICO_BNO08X_H
