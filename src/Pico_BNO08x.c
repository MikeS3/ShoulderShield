/*
 * Pico_BNO08x.c - Multi-IMU support for BNO08x on Raspberry Pi Pico
 */

#include "Pico_BNO08x.h"
#include <string.h>

#define MAX_BNO08X_IMUS 3

static bool bno08x_event_queue_push(bno08x_event_queue_t *queue, sh2_SensorValue_t *val) {
    uint8_t next = (queue->head + 1) % BNO08X_EVENT_QUEUE_SIZE;
    if (next == queue->tail) return false; // Queue full
    queue->queue[queue->head] = *val;
    queue->head = next;
    return true;
}

static bool bno08x_event_queue_pop(bno08x_event_queue_t *queue, sh2_SensorValue_t *val) {
    if (queue->head == queue->tail) return false; // Queue empty
    *val = queue->queue[queue->tail];
    queue->tail = (queue->tail + 1) % BNO08X_EVENT_QUEUE_SIZE;
    return true;
}

bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                           uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                           uint8_t cs_pin, uint8_t int_pin, uint8_t reset_pin,
                           uint32_t spi_speed) {
    bno->spi_port = spi_port;
    bno->cs_pin = cs_pin;
    bno->int_pin = int_pin;
    bno->reset_pin = reset_pin;
    bno->spi_speed = spi_speed;
    bno->interface_type = INTERFACE_SPI;

    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    gpio_init(reset_pin);
    gpio_set_dir(reset_pin, GPIO_OUT);
    gpio_put(reset_pin, 1);

    spi_init(spi_port, spi_speed);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);

    memset(&bno->event_queue, 0, sizeof(bno->event_queue));

    // TODO: Initialize BNO08x hardware & sh2 HAL for SPI
    return true; // stub for now
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    sh2_SensorValue_t val;
    while (pico_bno08x_get_sensor_event(bno, &val)) {
        bno08x_event_queue_push(&bno->event_queue, &val);
    }
}

bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value) {
    if (bno->event_queue.head == bno->event_queue.tail) {
        return false; // Queue is empty
    }

    *value = bno->event_queue.queue[bno->event_queue.tail];
    bno->event_queue.tail = (bno->event_queue.tail + 1) % BNO08X_EVENT_QUEUE_SIZE;
    return true;
}

// Add these functions to the end of your Pico_BNO08x.c file

bool multi_bno08x_init(Multi_BNO08x_t *multi) {
    if (!multi) return false;
    
    memset(multi, 0, sizeof(Multi_BNO08x_t));
    multi->imu_count = 0;
    
    return true;
}

bool multi_bno08x_add_spi_imu(Multi_BNO08x_t *multi, uint8_t index, spi_inst_t *spi_port,
                              uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                              uint8_t cs_pin, uint8_t int_pin, uint8_t reset_pin, uint32_t spi_speed) {
    if (!multi || index >= MAX_BNO08X_IMUS) return false;
    
    bool success = pico_bno08x_begin_spi(&multi->imus[index], spi_port, 
                                         miso_pin, mosi_pin, sck_pin, 
                                         cs_pin, int_pin, reset_pin, spi_speed);
    
    if (success) {
        multi->imus[index].initialized = true;
        if (index >= multi->imu_count) {
            multi->imu_count = index + 1;
        }
    }
    
    return success;
}

void multi_bno08x_service_all(Multi_BNO08x_t *multi) {
    if (!multi) return;
    
    for (uint8_t i = 0; i < multi->imu_count; i++) {
        if (multi->imus[i].initialized) {
            pico_bno08x_service(&multi->imus[i]);
        }
    }
}

bool multi_bno08x_get_sensor_event(Multi_BNO08x_t *multi, uint8_t imu_index, sh2_SensorValue_t *value) {
    if (!multi || !value || imu_index >= multi->imu_count) return false;
    
    if (!multi->imus[imu_index].initialized) return false;
    
    return pico_bno08x_get_sensor_event(&multi->imus[imu_index], value);
}

bool multi_bno08x_is_imu_initialized(Multi_BNO08x_t *multi, uint8_t imu_index) {
    if (!multi || imu_index >= multi->imu_count) return false;
    
    return multi->imus[imu_index].initialized;
}

uint8_t multi_bno08x_get_imu_count(Multi_BNO08x_t *multi) {
    if (!multi) return 0;
    
    return multi->imu_count;
}

void multi_bno08x_enable_all_reports(Multi_BNO08x_t *multi, uint8_t sensorId, uint32_t interval_us) {
    if (!multi) return;
    
    for (uint8_t i = 0; i < multi->imu_count; i++) {
        if (multi->imus[i].initialized) {
            // TODO: Implement sensor report enabling for each IMU
            // This would typically call sh2_setSensorConfig() for each IMU
            // For now, this is a stub
        }
    }
}
