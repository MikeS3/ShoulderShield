#ifndef _PICO_BNO08X_H
#define _PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define BNO08X_EVENT_QUEUE_SIZE 16

typedef enum {
    INTERFACE_NONE,
    INTERFACE_I2C,
    INTERFACE_SPI,
    INTERFACE_UART
} bno08x_interface_t;

typedef struct {
    sh2_SensorValue_t queue[BNO08X_EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
} bno08x_event_queue_t;

typedef struct {
    spi_inst_t *spi_port;
    uint8_t cs_pin;
    uint8_t int_pin;
    uint32_t spi_speed;

    bno08x_interface_t interface_type;

    bno08x_event_queue_t event_queue;
    sh2_Hal_t hal;
} Pico_BNO08x_t;

bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                           uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                           uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed);

void pico_bno08x_service(Pico_BNO08x_t *bno);
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);

// Multi-IMU helpers
void multi_bno08x_service_all(Pico_BNO08x_t *bno_array, uint8_t count);
bool multi_bno08x_get_sensor_event(Pico_BNO08x_t *bno_array, uint8_t index, sh2_SensorValue_t *value);

#endif
