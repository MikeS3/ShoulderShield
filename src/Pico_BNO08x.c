/*!
 *  @file Pico_BNO08x.c
 *
 *  Raspberry Pi Pico Driver for Multiple BNO08x 9-DOF Orientation IMU Fusion
 *
 *  Modified to support multiple IMU instances via SPI
 */

#include "Pico_BNO08x.h"
#include <string.h>

// Global variables for HAL callbacks - now arrays for multiple IMUs
static Pico_BNO08x_t *current_imus[MAX_IMU_COUNT] = {NULL, NULL, NULL};
static sh2_SensorValue_t *sensor_values[MAX_IMU_COUNT] = {NULL, NULL, NULL};
static bool reset_occurred[MAX_IMU_COUNT] = {false, false, false};

// Forward declarations for HAL functions
static bool pico_bno08x_init_common(Pico_BNO08x_t *bno);
static int spi_hal_open(sh2_Hal_t *self);
static void spi_hal_close(sh2_Hal_t *self);
static int spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hardware_reset(Pico_BNO08x_t *bno);
static Pico_BNO08x_t* get_current_imu_from_hal(sh2_Hal_t *hal);

/**
 * @brief Initialize BNO08x structure for individual IMU
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, uint8_t imu_id, int8_t reset_pin) {
    if (!bno || imu_id >= MAX_IMU_COUNT) return false;
    
    // Clear the structure
    memset(bno, 0, sizeof(Pico_BNO08x_t));
    
    // Set default values
    bno->imu_id = imu_id;
    bno->reset_pin = reset_pin;
    bno->int_pin = -1;
    bno->cs_pin = -1;
    bno->spi_speed = 1000000; // 1MHz
    bno->interface_type = INTERFACE_NONE;
    bno->reset_occurred = false;
    bno->initialized = false;
    
    // Setup HAL common functions
    bno->hal.getTimeUs = hal_get_time_us;
    
    return true;
}

/**
 * @brief Initialize SPI interface for individual IMU
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                          uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed) {
    if (!bno || !spi_port || bno->imu_id >= MAX_IMU_COUNT) return false;
    
    bno->spi_port = spi_port;
    bno->miso_pin = miso_pin;
    bno->mosi_pin = mosi_pin;
    bno->sck_pin = sck_pin;
    bno->cs_pin = cs_pin;
    bno->int_pin = int_pin;
    bno->spi_speed = spi_speed;
    bno->interface_type = INTERFACE_SPI;
    
    // Setup SPI pins (only if not already initialized)
    static bool spi_pins_initialized[2] = {false, false}; // For spi0 and spi1
    uint8_t spi_index = (spi_port == spi0) ? 0 : 1;
    
    if (!spi_pins_initialized[spi_index]) {
        gpio_set_function(miso_pin, GPIO_FUNC_SPI);
        gpio_set_function(sck_pin, GPIO_FUNC_SPI);
        gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
        
        // Initialize SPI (Mode 3: CPOL=1, CPHA=1)
        spi_init(spi_port, spi_speed);
        spi_set_format(spi_port, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi_pins_initialized[spi_index] = true;
    }
    
    // Setup CS pin (unique for each IMU)
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)
    
    // Setup interrupt pin (unique for each IMU)
    gpio_init(int_pin);
    gpio_set_dir(int_pin, GPIO_IN);
    gpio_pull_up(int_pin);
    
    // Setup HAL functions for SPI
    bno->hal.open = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read = spi_hal_read;
    bno->hal.write = spi_hal_write;
    
    return pico_bno08x_init_common(bno);
}

/**
 * @brief Common initialization after interface setup
 */
static bool pico_bno08x_init_common(Pico_BNO08x_t *bno) {
    if (bno->imu_id >= MAX_IMU_COUNT) return false;
    
    current_imus[bno->imu_id] = bno;
    
    // Perform hardware reset
    hardware_reset(bno);
    
    // Open SH2 interface
    int status = sh2_open(&bno->hal, hal_callback, (void*)(uintptr_t)bno->imu_id);
    if (status != SH2_OK) {
        printf("IMU %d: Failed to open SH2 interface: %d\n", bno->imu_id, status);
        return false;
    }
    
    // Get product IDs
    memset(&bno->prodIds, 0, sizeof(bno->prodIds));
    status = sh2_getProdIds(&bno->prodIds);
    if (status != SH2_OK) {
        printf("IMU %d: Failed to get product IDs: %d\n", bno->imu_id, status);
        return false;
    }
    
    // Register sensor callback
    sh2_setSensorCallback(sensor_handler, (void*)(uintptr_t)bno->imu_id);
    
    bno->initialized = true;
    printf("IMU %d: BNO08x initialized successfully\n", bno->imu_id);
    return true;
}

/**
 * @brief Hardware reset for individual IMU
 */
static void hardware_reset(Pico_BNO08x_t *bno) {
    if (bno->reset_pin >= 0) {
        gpio_init(bno->reset_pin);
        gpio_set_dir(bno->reset_pin, GPIO_OUT);
        gpio_put(bno->reset_pin, 1);
        sleep_ms(10);
        gpio_put(bno->reset_pin, 0);
        sleep_ms(10);
        gpio_put(bno->reset_pin, 1);
        sleep_ms(10);
    }
}

/**
 * @brief Get current IMU from HAL pointer
 */
static Pico_BNO08x_t* get_current_imu_from_hal(sh2_Hal_t *hal) {
    // Find which IMU this HAL belongs to
    for (int i = 0; i < MAX_IMU_COUNT; i++) {
        if (current_imus[i] && &current_imus[i]->hal == hal) {
            return current_imus[i];
        }
    }
    return NULL;
}

/**
 * @brief Enable sensor report for individual IMU
 */
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us) {
    if (!bno || !bno->initialized) return false;
    
    sh2_SensorConfig_t config;
    
    // Configure sensor
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(sensor_id, &config);
    return (status == SH2_OK);
}

/**
 * @brief Get sensor event for individual IMU
 */
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value) {
    if (!bno || !value || !bno->initialized) return false;
    
    sensor_values[bno->imu_id] = value;
    value->timestamp = 0;
    
    sh2_service();
    
    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        return false;
    }
    
    return true;
}

/**
 * @brief Service individual IMU
 */
void pico_bno08x_service(Pico_BNO08x_t *bno) {
    if (bno && bno->initialized) {
        sh2_service();
    }
}

/**
 * @brief Check if reset occurred for individual IMU
 */
bool pico_bno08x_was_reset(Pico_BNO08x_t *bno) {
    if (!bno) return false;
    
    bool reset = bno->reset_occurred;
    bno->reset_occurred = false;
    return reset;
}

/**
 * @brief Hardware reset for individual IMU (public function)
 */
void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno) {
    if (bno) {
        hardware_reset(bno);
    }
}

// Multi-IMU Management Functions

/**
 * @brief Initialize multi-IMU manager
 */
bool multi_bno08x_init(Multi_BNO08x_t *multi_bno) {
    if (!multi_bno) return false;
    
    memset(multi_bno, 0, sizeof(Multi_BNO08x_t));
    multi_bno->active_imu_count = 0;
    multi_bno->current_imu_index = 0;
    
    return true;
}

/**
 * @brief Add SPI IMU to multi-IMU manager
 */
bool multi_bno08x_add_spi_imu(Multi_BNO08x_t *multi_bno, uint8_t imu_index,
                             spi_inst_t *spi_port, uint8_t miso_pin, uint8_t mosi_pin, 
                             uint8_t sck_pin, uint8_t cs_pin, uint8_t int_pin, 
                             int8_t reset_pin, uint32_t spi_speed) {
    if (!multi_bno || imu_index >= MAX_IMU_COUNT) return false;
    
    // Initialize individual IMU
    if (!pico_bno08x_init(&multi_bno->imus[imu_index], imu_index, reset_pin)) {
        return false;
    }
    
    // Begin SPI for this IMU
    if (!pico_bno08x_begin_spi(&multi_bno->imus[imu_index], spi_port,
                              miso_pin, mosi_pin, sck_pin, cs_pin, int_pin, spi_speed)) {
        return false;
    }
    
    multi_bno08x->active_imu_count++;
    return true;
}

/**
 * @brief Enable reports for all IMUs
 */
bool multi_bno08x_enable_all_reports(Multi_BNO08x_t *multi_bno, sh2_SensorId_t sensor_id, uint32_t interval_us) {
    if (!multi_bno) return false;
    
    bool all_success = true;
    for (int i = 0; i < MAX_IMU_COUNT; i++) {
        if (multi_bno->imus[i].initialized) {
            if (!pico_bno08x_enable_report(&multi_bno->imus[i], sensor_id, interval_us)) {
                all_success = false;
                printf("Failed to enable sensor report for IMU %d\n", i);
            }
        }
    }
    
    return all_success;
}

/**
 * @brief Service all IMUs
 */
void multi_bno08x_service_all(Multi_BNO08x_t *multi_bno) {
    if (!multi_bno) return;
    
    for (int i = 0; i < MAX_IMU_COUNT; i++) {
        if (multi_bno->imus[i].initialized) {
            pico_bno08x_service(&multi_bno->imus[i]);
        }
    }
}

/**
 * @brief Get sensor event from specific IMU
 */
bool multi_bno08x_get_sensor_event(Multi_BNO08x_t *multi_bno, uint8_t imu_index, sh2_SensorValue_t *value) {
    if (!multi_bno || imu_index >= MAX_IMU_COUNT || !value) return false;
    
    if (!multi_bno->imus[imu_index].initialized) return false;
    
    return pico_bno08x_get_sensor_event(&multi_bno->imus[imu_index], value);
}

/**
 * @brief Get number of active IMUs
 */
uint8_t multi_bno08x_get_imu_count(Multi_BNO08x_t *multi_bno) {
    if (!multi_bno) return 0;
    return multi_bno->active_imu_count;
}

/**
 * @brief Check if specific IMU is initialized
 */
bool multi_bno08x_is_imu_initialized(Multi_BNO08x_t *multi_bno, uint8_t imu_index) {
    if (!multi_bno || imu_index >= MAX_IMU_COUNT) return false;
    return multi_bno->imus[imu_index].initialized;
}

// HAL Implementation for SPI (modified to work with multiple IMUs)
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    for (int i = 0; i < 500; i++) {
        if (!gpio_get(bno->int_pin)) return true;
        sleep_ms(1);
    }
    hardware_reset(bno);
    return false;
}

static int spi_hal_open(sh2_Hal_t *self) {
    Pico_BNO08x_t *bno = get_current_imu_from_hal(self);
    if (!bno) return -1;
    
    return spi_hal_wait_for_int(bno) ? 0 : -1;
}

static void spi_hal_close(sh2_Hal_t *self) {
    // Nothing special to do for SPI close
}

static int spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    Pico_BNO08x_t *bno = get_current_imu_from_hal(self);
    if (!bno || !pBuffer) return 0;
    
    if (!spi_hal_wait_for_int(bno)) return 0;
    
    uint8_t dummy = 0x00;
    
    // Read header first
    gpio_put(bno->cs_pin, 0);
    spi_read_blocking(bno->spi_port, dummy, pBuffer, 4);
    gpio_put(bno->cs_pin, 1);
    
    uint16_t packet_size = (uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8);
    packet_size &= ~0x8000; // Clear continuation bit
    
    if (packet_size > len) return 0;
    
    if (!spi_hal_wait_for_int(bno)) return 0;
    
    // Read full packet
    gpio_put(bno->cs_pin, 0);
    spi_read_blocking(bno->spi_port, dummy, pBuffer, packet_size);
    gpio_put(bno->cs_pin, 1);
    
    return packet_size;
}

static int spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    Pico_BNO08x_t *bno = get_current_imu_from_hal(self);
    if (!bno || !pBuffer) return 0;
    
    if (!spi_hal_wait_for_int(bno)) return 0;
    
    gpio_put(bno->cs_pin, 0);
    spi_write_blocking(bno->spi_port, pBuffer, len);
    gpio_put(bno->cs_pin, 1);
    
    return len;
}

// Common HAL functions
static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    return time_us_32();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    uint8_t imu_id = (uint8_t)(uintptr_t)cookie;
    
    if (pEvent->eventId == SH2_RESET) {
        reset_occurred[imu_id] = true;
        if (current_imus[imu_id]) {
            current_imus[imu_id]->reset_occurred = true;
        }
    }
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    uint8_t imu_id = (uint8_t)(uintptr_t)cookie;
    
    if (imu_id >= MAX_IMU_COUNT || !sensor_values[imu_id]) return;
    
    int rc = sh2_decodeSensorEvent(sensor_values[imu_id], event);
    if (rc != SH2_OK) {
        printf("IMU %d: Error decoding sensor event: %d\n", imu_id, rc);
        sensor_values[imu_id]->timestamp = 0;
    }
}
