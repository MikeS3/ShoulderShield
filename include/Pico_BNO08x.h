#ifndef PICO_BNO08X_H
#define PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO08x_I2CADDR_DEFAULT 0x4A

typedef enum {
    INTERFACE_NONE = 0,
    INTERFACE_SPI,
    INTERFACE_I2C
} interface_type_t;

typedef struct {
    // Common fields
    int8_t reset_pin;
    int instance_id;
    interface_type_t interface_type;
    
    // SPI fields
    spi_inst_t *spi_port;
    uint8_t miso_pin, mosi_pin, sck_pin;
    uint8_t cs_pin, int_pin;
    uint32_t spi_speed;
    
    // I2C fields  
    i2c_inst_t *i2c_port;
    uint8_t sda_pin, scl_pin;
    uint8_t i2c_addr;
    uint32_t i2c_speed;

    // SH2 HAL (each IMU gets its own completely separate instance)
    sh2_Hal_t hal;
    
    // Sensor data
    sh2_SensorValue_t sensor_value;
    sh2_SensorValue_t *pending_value;
    
    // State
    bool has_reset;
    bool initialized;
} Pico_BNO08x_t;

// Function prototypes - simplified API
bool pico_bno08x_init_spi(Pico_BNO08x_t *bno, int8_t reset_pin, int instance_id,
                          spi_inst_t *spi_port, uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed);

bool pico_bno08x_init_i2c(Pico_BNO08x_t *bno, int8_t reset_pin, int instance_id,
                          i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin, 
                          uint8_t i2c_addr, uint32_t i2c_speed);

bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us);
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);
void pico_bno08x_service(Pico_BNO08x_t *bno);

#ifdef __cplusplus
}
#endif

#endif // PICO_BNO08X_H