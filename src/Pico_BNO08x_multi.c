/* Pico_BNO08x_multi.c - Multi-IMU SPI/I2C driver for Raspberry Pi Pico */

#include <stdio.h>
#include "Pico_BNO08x.h"
#include <string.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

// Forward declarations
static void hardware_reset(Pico_BNO08x_t *bno);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *e);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *e);

// SPI HAL functions
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno);
static int spi_hal_open(sh2_Hal_t *self);
static void spi_hal_close(sh2_Hal_t *self);
static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us);
static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len);

// I2C HAL functions
static int i2c_hal_open(sh2_Hal_t *self);
static void i2c_hal_close(sh2_Hal_t *self);
static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

bool pico_bno08x_init(Pico_BNO08x_t *bno, int reset_pin, int instance_id) {
    if (!bno) return false;
    memset(bno, 0, sizeof(*bno));
    bno->reset_pin = reset_pin;
    bno->instance_id = instance_id;
    bno->spi_frequency = 1000000;
    bno->i2c_speed = 400000;
    bno->i2c_addr = BNO08x_I2CADDR_DEFAULT;
    bno->interface_type = INTERFACE_NONE;
    bno->has_reset = false;
    bno->reset_occurred = false;
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
    bno->interface_type = INTERFACE_SPI;
    
    // Setup CS pin
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1); // CS high (inactive)
    
    // Setup interrupt pin
    gpio_init(irq);
    gpio_set_dir(irq, GPIO_IN);
    gpio_pull_up(irq);
    
    // Setup HAL functions for SPI
    bno->hal.open = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read = spi_hal_read;
    bno->hal.write = spi_hal_write;
    
    // Initialize SH2 layer
    sh2_open(&bno->hal, hal_callback, bno);
    sh2_setSensorCallback(sensor_handler, bno);
    
    return true;
}

bool pico_bno08x_begin_i2c(Pico_BNO08x_t *bno, i2c_inst_t *i2c_port,
                           uint8_t sda_pin, uint8_t scl_pin,
                           uint8_t i2c_addr, uint32_t i2c_speed) {
    if (!bno || !i2c_port) return false;
    
    bno->i2c_port = i2c_port;
    bno->sda_pin = sda_pin;
    bno->scl_pin = scl_pin;
    bno->i2c_addr = i2c_addr;
    bno->i2c_speed = i2c_speed;
    bno->interface_type = INTERFACE_I2C;
    
    // Setup I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // Initialize I2C
    i2c_init(i2c_port, i2c_speed);
    
    // Setup HAL functions for I2C
    bno->hal.open = i2c_hal_open;
    bno->hal.close = i2c_hal_close;
    bno->hal.read = i2c_hal_read;
    bno->hal.write = i2c_hal_write;
    
    // Initialize SH2 layer
    sh2_open(&bno->hal, hal_callback, bno);
    sh2_setSensorCallback(sensor_handler, bno);
    
    return true;
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    if (!bno) return;
    
    // Service the SH2 layer for this instance
    sh2_service();
}

void hardware_reset(Pico_BNO08x_t *bno) {
    if (!bno || bno->reset_pin < 0) return;
    
    gpio_init(bno->reset_pin);
    gpio_set_dir(bno->reset_pin, GPIO_OUT);
    gpio_put(bno->reset_pin, 0);
    sleep_ms(10);
    gpio_put(bno->reset_pin, 1);
    sleep_ms(50);
    bno->has_reset = true;
}

bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us) {
    if (!bno) return false;
    
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled = false,
        .wakeupEnabled = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled = false,
        .changeSensitivity = 0,
        .reportInterval_us = interval_us,
        .batchInterval_us = 0
    };
    return (sh2_setSensorConfig(sensor_id, &cfg) == SH2_OK);
}

bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value) {
    if (!bno || !value) return false;
    
    if (bno->pending_value->timestamp == 0 && 
        bno->pending_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        return false;
    }
    
    *value = *bno->pending_value;
    bno->pending_value->timestamp = 0; // Mark as consumed
    return true;
}

// HAL callback functions
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

// SPI HAL Implementation
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    for (int i = 0; i < 500; i++) {
        if (!gpio_get(bno->int_pin)) return true;
        sleep_ms(1);
    }
    hardware_reset(bno);
    return false;
}

static int spi_hal_open(sh2_Hal_t *self) { 
    (void)self; 
    return SH2_OK; 
}

static void spi_hal_close(sh2_Hal_t *self) { 
    (void)self; 
}

static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    gpio_put(bno->cs_pin, 0);
    int ret = spi_read_blocking(bno->spi_port, 0x00, buf, len);
    gpio_put(bno->cs_pin, 1);
    if (t_us) *t_us = time_us_32();
    return ret;
}

static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    gpio_put(bno->cs_pin, 0);
    int ret = spi_write_blocking(bno->spi_port, buf, len);
    gpio_put(bno->cs_pin, 1);
    return ret;
}

// I2C HAL Implementation
static int i2c_hal_open(sh2_Hal_t *self) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    if (!bno) return -1;
    
    // Send soft reset packet
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    int result = i2c_write_blocking(bno->i2c_port, bno->i2c_addr, 
                                   softreset_pkt, 5, false);
    if (result < 0) return -1;
    
    sleep_ms(300);
    return 0;
}

static void i2c_hal_close(sh2_Hal_t *self) {
    // Nothing to do for I2C close
    (void)self;
}

static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    if (!bno || !pBuffer) return 0;
    
    uint8_t header[4];
    int result = i2c_read_blocking(bno->i2c_port, bno->i2c_addr, 
                                  header, 4, false);
    if (result < 0) return 0;
    
    uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_size &= ~0x8000; // Clear continuation bit
    
    if (packet_size > len) return 0;
    
    // Read the rest of the packet
    uint16_t cargo_remaining = packet_size;
    uint16_t read_pos = 0;
    
    // Copy header first
    if (cargo_remaining >= 4) {
        memcpy(pBuffer, header, 4);
        read_pos = 4;
        cargo_remaining -= 4;
    }
    
    // Read remaining data
    while (cargo_remaining > 0) {
        uint16_t to_read = (cargo_remaining > 252) ? 252 : cargo_remaining;
        result = i2c_read_blocking(bno->i2c_port, bno->i2c_addr,
                                  pBuffer + read_pos, to_read, false);
        if (result < 0) return 0;
        
        read_pos += to_read;
        cargo_remaining -= to_read;
    }
    
    if (t_us) *t_us = time_us_32();
    return packet_size;
}

static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    if (!bno || !pBuffer) return 0;
    
    int result = i2c_write_blocking(bno->i2c_port, bno->i2c_addr,
                                   pBuffer, len, false);
    return (result < 0) ? 0 : len;
}