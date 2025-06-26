#ifndef _PICO_BNO08X_H
#define _PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define BNO08x_I2CADDR_DEFAULT 0x4A

typedef struct {
    spi_inst_t *spi_port;
    uint8_t miso_pin, mosi_pin, sck_pin;
    uint8_t cs_pin, int_pin, reset_pin;
    uint32_t spi_speed;

    sh2_Hal_t hal;
    sh2_ProductIds_t prodIds;
    bool reset_occurred;
} Pico_BNO08x_t;

// single‑IMU APIs
bool pico_bno08x_init      (Pico_BNO08x_t *bno, int8_t reset_pin);
bool pico_bno08x_begin_spi (Pico_BNO08x_t *bno,
                            spi_inst_t *spi_port,
                            uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                            uint8_t cs_pin,   uint8_t int_pin,
                            uint32_t spi_speed);
void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno);
bool pico_bno08x_was_reset (Pico_BNO08x_t *bno);
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno,
                               sh2_SensorId_t sensor_id,
                               uint32_t interval_us);
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno,
                                  sh2_SensorValue_t *value);
void pico_bno08x_service   (Pico_BNO08x_t *bno);

// multi‑IMU “active” selector
void pico_bno08x_set_active(Pico_BNO08x_t *bno);

#endif // _PICO_BNO08X_H
