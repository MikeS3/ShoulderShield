/* Pico_BNO08x_multi.c - Simple 3-IMU driver using working single-IMU pattern */

#include "Pico_BNO08x.h"
#include <string.h>
#include <stddef.h>

#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

// Forward declarations
static void hardware_reset(Pico_BNO08x_t *bno);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *e);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *e);

// SPI HAL functions
static int spi_hal_open(sh2_Hal_t *self);
static void spi_hal_close(sh2_Hal_t *self);
static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us);
static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len);

// I2C HAL functions
static int i2c_hal_open(sh2_Hal_t *self);
static void i2c_hal_close(sh2_Hal_t *self);
static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

bool pico_bno08x_init_spi(Pico_BNO08x_t *bno, int8_t reset_pin, int instance_id,
                          spi_inst_t *spi_port, uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed) {
    if (!bno || !spi_port) return false;
    
    // Clear structure
    memset(bno, 0, sizeof(*bno));
    
    // Set basic info
    bno->reset_pin = reset_pin;
    bno->instance_id = instance_id;
    bno->interface_type = INTERFACE_SPI;
    bno->pending_value = &bno->sensor_value;
    
    // Set SPI info
    bno->spi_port = spi_port;
    bno->miso_pin = miso_pin;
    bno->mosi_pin = mosi_pin;
    bno->sck_pin = sck_pin;
    bno->cs_pin = cs_pin;
    bno->int_pin = int_pin;
    bno->spi_speed = spi_speed;
    
    // Initialize SPI bus (using working mainOld.c pattern)
    static bool spi0done = false, spi1done = false;
    if (spi_port == spi0 && !spi0done) {
        gpio_set_function(miso_pin, GPIO_FUNC_SPI);
        gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(sck_pin, GPIO_FUNC_SPI);
        spi_init(spi0, spi_speed);
        spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi0done = true;
    }
    if (spi_port == spi1 && !spi1done) {
        gpio_set_function(miso_pin, GPIO_FUNC_SPI);
        gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(sck_pin, GPIO_FUNC_SPI);
        spi_init(spi1, spi_speed);
        spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi1done = true;
    }
    
    // Setup pins
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)
    
    gpio_init(int_pin);
    gpio_set_dir(int_pin, GPIO_IN);
    gpio_pull_up(int_pin);
    
    // Hardware reset
    hardware_reset(bno);
    
    // Setup HAL functions
    bno->hal.getTimeUs = hal_get_time_us;
    bno->hal.open = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read = spi_hal_read;
    bno->hal.write = spi_hal_write;
    
    // Initialize SH2 (each IMU gets its own separate SH2 instance - this is the key!)
    if (sh2_open(&bno->hal, hal_callback, bno) != SH2_OK) {
        printf("Failed to open SH2 for IMU %d\n", instance_id);
        return false;
    }
    sh2_setSensorCallback(sensor_handler, bno);
    
    printf("IMU %d (SPI) initialized successfully\n", instance_id);
    bno->initialized = true;
    return true;
}

bool pico_bno08x_init_i2c(Pico_BNO08x_t *bno, int8_t reset_pin, int instance_id,
                          i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin, 
                          uint8_t i2c_addr, uint32_t i2c_speed) {
    if (!bno || !i2c_port) return false;
    
    // Clear structure
    memset(bno, 0, sizeof(*bno));
    
    // Set basic info
    bno->reset_pin = reset_pin;
    bno->instance_id = instance_id;
    bno->interface_type = INTERFACE_I2C;
    bno->pending_value = &bno->sensor_value;
    
    // Set I2C info
    bno->i2c_port = i2c_port;
    bno->sda_pin = sda_pin;
    bno->scl_pin = scl_pin;
    bno->i2c_addr = i2c_addr;
    bno->i2c_speed = i2c_speed;
    
    // Setup I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // Initialize I2C
    i2c_init(i2c_port, i2c_speed);
    
    // Hardware reset
    hardware_reset(bno);
    
    // Setup HAL functions
    bno->hal.getTimeUs = hal_get_time_us;
    bno->hal.open = i2c_hal_open;
    bno->hal.close = i2c_hal_close;
    bno->hal.read = i2c_hal_read;
    bno->hal.write = i2c_hal_write;
    
    // Initialize SH2 (each IMU gets its own separate SH2 instance - this is the key!)
    if (sh2_open(&bno->hal, hal_callback, bno) != SH2_OK) {
        printf("Failed to open SH2 for IMU %d\n", instance_id);
        return false;
    }
    sh2_setSensorCallback(sensor_handler, bno);
    
    printf("IMU %d (I2C) initialized successfully\n", instance_id);
    bno->initialized = true;
    return true;
}

bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us) {
    if (!bno || !bno->initialized) return false;
    
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled = false,
        .wakeupEnabled = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled = false,
        .changeSensitivity = 0,
        .reportInterval_us = interval_us,
        .batchInterval_us = 0
    };
    
    int result = sh2_setSensorConfig(sensor_id, &cfg);
    printf("IMU %d sensor %d config: %s\n", bno->instance_id, sensor_id, 
           (result == SH2_OK) ? "OK" : "FAILED");
    
    return (result == SH2_OK);
}

// This is the key function from mainOld.c that actually works!
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *val) {
    if (!bno || !val || !bno->initialized) return false;
    
    // This is the exact working pattern from mainOld.c
    bno->pending_value = val;
    val->timestamp = 0;
    sh2_service();  // Each IMU calls its own sh2_service()
    bno->pending_value = &bno->sensor_value;
    
    return (val->timestamp != 0 || val->sensorId == SH2_GYRO_INTEGRATED_RV);
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    if (!bno || !bno->initialized) return;
    sh2_service();  // Each IMU services its own SH2 instance
}

// HAL implementation
static void hardware_reset(Pico_BNO08x_t *bno) {
    if (!bno || bno->reset_pin < 0) return;
    
    gpio_init(bno->reset_pin);
    gpio_set_dir(bno->reset_pin, GPIO_OUT);
    gpio_put(bno->reset_pin, 0);
    sleep_ms(10);
    gpio_put(bno->reset_pin, 1);
    sleep_ms(50);
    bno->has_reset = true;
}

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
    return SH2_OK;
}

static void i2c_hal_close(sh2_Hal_t *self) {
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