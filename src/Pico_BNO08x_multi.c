/* Pico_BNO08x_multi.c - Multi-IMU SPI driver for Raspberry Pi Pico */

#include <stdio.h>
#include "Pico_BNO08x.h"
#include <string.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

static void hardware_reset(Pico_BNO08x_t *bno);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *e);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *e);

static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno);
static int spi_hal_open(sh2_Hal_t *self);
static void spi_hal_close(sh2_Hal_t *self);
static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us);
static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len);

bool pico_bno08x_init(Pico_BNO08x_t *bno, int reset_pin, int instance_id) {
    if (!bno) return false;
    memset(bno, 0, sizeof(*bno));
    bno->reset_pin = reset_pin;
    bno->instance_id = instance_id;
    bno->spi_frequency = 1000000;
    bno->has_reset = false;
    bno->hal.getTimeUs = hal_get_time_us;
    bno->hal.open = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read = spi_hal_read;
    bno->hal.write = spi_hal_write;
    bno->hal.getTimeUs = hal_get_time_us;
    bno->pending_value = &bno->sensor_value;
    return true;
}

bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno,
                           spi_inst_t *spi,
                           uint8_t miso, uint8_t mosi, uint8_t sck,
                           uint8_t cs, uint8_t irq,
                           uint32_t speed) {
    if (!bno || !spi) return false;
    bno->spi_port = spi;
    bno->miso_pin = miso;
    bno->mosi_pin = mosi;
    bno->sck_pin = sck;
    bno->cs_pin = cs;
    bno->int_pin = irq;
    bno->spi_frequency = speed;

    static bool spi0done = false, spi1done = false;
    if (spi == spi0 && !spi0done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi0, speed);
        spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi0done = true;
    }
    if (spi == spi1 && !spi1done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi1, speed);
        spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi1done = true;
    }

    gpio_init(cs);    gpio_set_dir(cs, GPIO_OUT); gpio_put(cs, 1);
    gpio_init(irq);   gpio_set_dir(irq, GPIO_IN); gpio_pull_up(irq);
    gpio_put(bno->cs_pin, 1);               // make sure CS is high

    hardware_reset(bno);
    sleep_ms(200);    // give the chip 200 ms to come fully alive
    printf("[DEBUG] IMU%d: reset complete, now sh2_open()\n", bno->instance_id);
        int rc = sh2_open(&bno->hal, hal_callback, bno);
    printf("[DEBUG] IMU%d: sh2_open returned %d\n", bno->instance_id, rc);
    if (rc != SH2_OK) {
        return false;
    }
    sh2_setSensorCallback(sensor_handler, bno);
    printf("IMU %d initialized\n", bno->instance_id);
    return true;    // <-- GOOD: returns true on SH2_OK
}


bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *val) {
    if (!bno || !val) return false;
    bno->pending_value = val;
    val->timestamp = 0;
    sh2_service();
    bno->pending_value = &bno->sensor_value;
    return (val->timestamp != 0 || val->sensorId == SH2_GYRO_INTEGRATED_RV);
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    sh2_service();
}

bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t id, uint32_t iu) {
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled  = false,
        .wakeupEnabled             = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled           = false,
        .changeSensitivity         = 0,
        .batchInterval_us          = 0,
        .sensorSpecific            = 0,
        .reportInterval_us         = iu
    };
    return (sh2_setSensorConfig(id, &cfg) == SH2_OK);
}

// HAL callbacks
static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    (void)self;
    return time_us_32();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *e) {
    Pico_BNO08x_t *bno = (Pico_BNO08x_t *)cookie;
    if (e->eventId == SH2_RESET) bno->has_reset = true;
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *evt) {
    Pico_BNO08x_t *bno = (Pico_BNO08x_t *)cookie;
    if (sh2_decodeSensorEvent(bno->pending_value, evt) != SH2_OK)
        bno->pending_value->timestamp = 0;
}

static int spi_hal_open(sh2_Hal_t *self) { (void)self; return SH2_OK; }
static void spi_hal_close(sh2_Hal_t *self) { (void)self; }
#define INT_TIMEOUT_MS 2000
#define INT_STABLE_MS    10

static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    int stable = 0;
    for (int i = 0; i < INT_TIMEOUT_MS; i++) {
        if (!gpio_get(bno->int_pin)) {
            if (++stable >= INT_STABLE_MS) return true;
        } else {
            stable = 0;
        }
        sleep_ms(1);
    }
    printf("[WARN] IMU%d: INT never stayed low for %d ms\n",
           bno->instance_id, INT_TIMEOUT_MS);
    hardware_reset(bno);
    return false;
}

static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    if (!spi_hal_wait_for_int(bno)) {
    printf("[WARN] IMU%d: timeout waiting for INT before SPI\n", bno->instance_id);
    return 0;
}

    gpio_put(bno->cs_pin, 0);
    int ret = spi_read_blocking(bno->spi_port, 0x00, buf, len);
    gpio_put(bno->cs_pin, 1);
    if (t_us) *t_us = time_us_32();
    return ret;
}

static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    if (!spi_hal_wait_for_int(bno)) {
    printf("[WARN] IMU%d: timeout waiting for INT before SPI\n", bno->instance_id);
    return 0;
}

    gpio_put(bno->cs_pin, 0);
    int ret = spi_write_blocking(bno->spi_port, buf, len);
    gpio_put(bno->cs_pin, 1);
    return ret;
}

void hardware_reset(Pico_BNO08x_t *bno) {
    gpio_init(bno->reset_pin);
    gpio_set_dir(bno->reset_pin, GPIO_OUT);
    gpio_put(bno->reset_pin, 0);
    sleep_ms(20);
    gpio_put(bno->reset_pin, 1);
    sleep_ms(100);
}

void pico_bno08x_reset(Pico_BNO08x_t *bno) {
    hardware_reset(bno);
}