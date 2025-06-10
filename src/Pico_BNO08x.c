/*
 * Pico_BNO08x.c - Multi-IMU support for BNO08x on Raspberry Pi Pico
 */

#include "Pico_BNO08x.h"
#include <string.h>

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
                           uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed) {
    bno->spi_port = spi_port;
    bno->cs_pin = cs_pin;
    bno->int_pin = int_pin;
    bno->spi_speed = spi_speed;
    bno->interface_type = INTERFACE_SPI;

    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    spi_init(spi_port, spi_speed);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);

    memset(&bno->event_queue, 0, sizeof(bno->event_queue));

    // TODO: Implement HAL init for BNO08x SH2 with SPI here
    return true; // Stubbed for now
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    sh2_SensorValue_t val;
    while (pico_bno08x_get_sensor_event(bno, &val)) {
        bno08x_event_queue_push(&bno->event_queue, &val);
    }
}

bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value) {
    return bno08x_event_queue_pop(&bno->event_queue, value);
}

void multi_bno08x_service_all(Pico_BNO08x_t *bno_array, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        pico_bno08x_service(&bno_array[i]);
    }
}

bool multi_bno08x_get_sensor_event(Pico_BNO08x_t *bno_array, uint8_t index, sh2_SensorValue_t *value) {
    return pico_bno08x_get_sensor_event(&bno_array[index], value);
}
