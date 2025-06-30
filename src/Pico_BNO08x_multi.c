/* Pico_BNO08x.c - Multi-IMU SPI driver - ROBUST VERSION */

#include <stdio.h>
#include "Pico_BNO08x.h"
#include <string.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Timeout constants
#define INT_TIMEOUT_MS 3000      // Increased timeout
#define INT_STABLE_MS    20      // More stable time
#define RESET_DELAY_MS   50      // Longer reset
#define BOOT_DELAY_MS    500     // Longer boot delay
#define RETRY_DELAY_MS   200     // Delay between retries

// Get containing structure from member pointer
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

// Forward declarations
static void hardware_reset(Pico_BNO08x_t *bno);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *e);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *e);
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno);
static int spi_hal_open(sh2_Hal_t *self);
static void spi_hal_close(sh2_Hal_t *self);
static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us);
static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len);

/**
 * @brief Initialize BNO08x instance
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, int reset_pin, int instance_id) {
    if (!bno) {
        printf("[ERROR] NULL BNO08x pointer\n");
        return false;
    }
    
    // Clear the structure
    memset(bno, 0, sizeof(*bno));
    
    // Set basic parameters
    bno->reset_pin = reset_pin;
    bno->instance_id = instance_id;
    bno->spi_frequency = 1000000;  // 1MHz default
    bno->has_reset = false;
    
    // Create dedicated SH2 instance for this IMU
    bno->sh2_instance = sh2_createInstance();
    if (!bno->sh2_instance) {
        printf("[ERROR] IMU%d: Failed to create SH2 instance\n", instance_id);
        return false;
    }
    
    // Setup HAL functions
    bno->hal.getTimeUs = hal_get_time_us;
    bno->hal.open = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read = spi_hal_read;
    bno->hal.write = spi_hal_write;
    
    // Initialize pending value pointer
    bno->pending_value = &bno->sensor_value;
    
    printf("[DEBUG] IMU%d: Initialized with dedicated SH2 instance\n", instance_id);
    return true;
}

/**
 * @brief Start SPI communication
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno,
                           spi_inst_t *spi,
                           uint8_t miso, uint8_t mosi, uint8_t sck,
                           uint8_t cs, uint8_t irq,
                           uint32_t speed) {
    if (!bno || !spi || !bno->sh2_instance) {
        printf("[ERROR] Invalid parameters to begin_spi\n");
        return false;
    }
    
    // Store SPI configuration
    bno->spi_port = spi;
    bno->miso_pin = miso;
    bno->mosi_pin = mosi;
    bno->sck_pin = sck;
    bno->cs_pin = cs;
    bno->int_pin = irq;
    bno->spi_frequency = speed;

    // Initialize SPI bus (only once per SPI instance)
    static bool spi0_done = false, spi1_done = false;
    if (spi == spi0 && !spi0_done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi0, speed);
        spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi0_done = true;
        printf("[DEBUG] IMU%d: Initialized SPI0 bus\n", bno->instance_id);
    }
    if (spi == spi1 && !spi1_done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi1, speed);
        spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi1_done = true;
        printf("[DEBUG] IMU%d: Initialized SPI1 bus\n", bno->instance_id);
    }

    // Setup individual pins for this IMU
    gpio_init(cs);    
    gpio_set_dir(cs, GPIO_OUT); 
    gpio_put(cs, 1);  // CS high (inactive)
    
    gpio_init(irq);   
    gpio_set_dir(irq, GPIO_IN); 
    gpio_pull_up(irq);

    printf("[DEBUG] IMU%d: GPIO pins configured (CS=%d, INT=%d)\n", 
           bno->instance_id, cs, irq);

    // Perform hardware reset
    printf("[DEBUG] IMU%d: Starting hardware reset sequence\n", bno->instance_id);
    hardware_reset(bno);
    sleep_ms(BOOT_DELAY_MS);
    
    // Check if interrupt pin is responsive
    printf("[DEBUG] IMU%d: Checking INT pin state: %d\n", 
           bno->instance_id, gpio_get(bno->int_pin));
    
    printf("[DEBUG] IMU%d: Opening SH2 instance\n", bno->instance_id);
    
    // Clear reset flag before opening
    bno->has_reset = false;
    
    // Open SH2 instance with this IMU's dedicated instance
    int rc = sh2_openInstance(bno->sh2_instance, &bno->hal, hal_callback, bno);
    printf("[DEBUG] IMU%d: sh2_openInstance returned code %d\n", bno->instance_id, rc);
    
    if (rc != SH2_OK) {
        printf("[ERROR] IMU%d: sh2_openInstance failed with code %d\n", bno->instance_id, rc);
        
        // Try to diagnose the issue
        printf("[DEBUG] IMU%d: Diagnosing failure...\n", bno->instance_id);
        printf("[DEBUG] IMU%d: - Reset pin: %d\n", bno->instance_id, bno->reset_pin);
        printf("[DEBUG] IMU%d: - CS pin: %d (state: %d)\n", bno->instance_id, bno->cs_pin, gpio_get(bno->cs_pin));
        printf("[DEBUG] IMU%d: - INT pin: %d (state: %d)\n", bno->instance_id, bno->int_pin, gpio_get(bno->int_pin));
        printf("[DEBUG] IMU%d: - Has reset: %s\n", bno->instance_id, bno->has_reset ? "YES" : "NO");
        
        // Try one more reset and retry
        printf("[DEBUG] IMU%d: Attempting recovery reset...\n", bno->instance_id);
        hardware_reset(bno);
        sleep_ms(BOOT_DELAY_MS * 2);  // Double the boot delay
        
        bno->has_reset = false;
        rc = sh2_openInstance(bno->sh2_instance, &bno->hal, hal_callback, bno);
        printf("[DEBUG] IMU%d: Recovery attempt returned code %d\n", bno->instance_id, rc);
        
        if (rc != SH2_OK) {
            return false;
        }
    }
    
    // Set sensor callback for this instance
    int cb_rc = sh2_setSensorCallbackInstance(bno->sh2_instance, sensor_handler, bno);
    printf("[DEBUG] IMU%d: Sensor callback set, code %d\n", bno->instance_id, cb_rc);
    
    bno->initialized = true;
    printf("[SUCCESS] IMU%d: Fully initialized with SH2 instance\n", bno->instance_id);
    return true;
}

/**
 * @brief Service the IMU (must be called regularly)
 */
void pico_bno08x_service(Pico_BNO08x_t *bno) {
    if (!bno || !bno->sh2_instance) return;
    sh2_serviceInstance(bno->sh2_instance);
}

/**
 * @brief Enable sensor report
 */
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t id, uint32_t interval_us) {
    if (!bno || !bno->sh2_instance) {
        printf("[ERROR] Invalid BNO08x instance for enable_report\n");
        return false;
    }
    
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled  = false,
        .wakeupEnabled             = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled           = false,
        .sniffEnabled              = false,
        .changeSensitivity         = 0,
        .batchInterval_us          = 0,
        .sensorSpecific            = 0,
        .reportInterval_us         = interval_us
    };
    
    int status = sh2_setSensorConfigInstance(bno->sh2_instance, id, &cfg);
    printf("[DEBUG] IMU%d: Enable sensor %d, status=%d\n", bno->instance_id, id, status);
    return (status == SH2_OK);
}

/**
 * @brief Disable sensor report
 */
bool pico_bno08x_disable_report(Pico_BNO08x_t *bno, sh2_SensorId_t id) {
    if (!bno || !bno->sh2_instance) return false;
    
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled  = false,
        .wakeupEnabled             = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled           = false,
        .sniffEnabled              = false,
        .changeSensitivity         = 0,
        .batchInterval_us          = 0,
        .sensorSpecific            = 0,
        .reportInterval_us         = 0  // 0 = disable
    };
    
    return (sh2_setSensorConfigInstance(bno->sh2_instance, id, &cfg) == SH2_OK);
}

/**
 * @brief Get sensor event
 */
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *val) {
    if (!bno || !val || !bno->sh2_instance) return false;
    
    bno->pending_value = val;
    val->timestamp = 0;
    
    // Service this specific instance
    sh2_serviceInstance(bno->sh2_instance);
    
    // Restore default pending value
    bno->pending_value = &bno->sensor_value;
    
    return (val->timestamp != 0 || val->sensorId == SH2_GYRO_INTEGRATED_RV);
}

/**
 * @brief Check if data is available
 */
bool pico_bno08x_data_available(Pico_BNO08x_t *bno) {
    if (!bno || !bno->sh2_instance) return false;
    return bno->sensor_value.timestamp != 0;
}

/**
 * @brief Hardware reset
 */
void pico_bno08x_reset(Pico_BNO08x_t *bno) {
    if (!bno) return;
    hardware_reset(bno);
}

/**
 * @brief Cleanup and destroy instance
 */
void pico_bno08x_destroy(Pico_BNO08x_t *bno) {
    if (!bno) return;
    
    if (bno->sh2_instance) {
        sh2_closeInstance(bno->sh2_instance);
        sh2_destroyInstance(bno->sh2_instance);
        bno->sh2_instance = NULL;
    }
    
    printf("[DEBUG] IMU%d: Destroyed\n", bno->instance_id);
}

// ===== HAL Implementation =====

static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    (void)self;
    return time_us_32();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *e) {
    Pico_BNO08x_t *bno = (Pico_BNO08x_t *)cookie;
    if (e->eventId == SH2_RESET) {
        bno->has_reset = true;
        printf("[DEBUG] IMU%d: Reset event received ✓\n", bno->instance_id);
    }
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *evt) {
    Pico_BNO08x_t *bno = (Pico_BNO08x_t *)cookie;
    if (sh2_decodeSensorEvent(bno->pending_value, evt) != SH2_OK) {
        bno->pending_value->timestamp = 0;
    }
}

static int spi_hal_open(sh2_Hal_t *self) { 
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    printf("[DEBUG] IMU%d: HAL open called\n", bno->instance_id);
    return SH2_OK; 
}

static void spi_hal_close(sh2_Hal_t *self) { 
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    printf("[DEBUG] IMU%d: HAL close called\n", bno->instance_id);
}

static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    int stable = 0;
    for (int i = 0; i < INT_TIMEOUT_MS; i++) {
        if (!gpio_get(bno->int_pin)) {
            if (++stable >= INT_STABLE_MS) {
                return true;
            }
        } else {
            stable = 0;
        }
        sleep_ms(1);
    }
    printf("[WARN] IMU%d: INT timeout after %dms\n", bno->instance_id, INT_TIMEOUT_MS);
    return false;  // Don't reset on timeout for now
}

static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    
    // For now, let's not wait for INT during initialization to see if that helps
    // We can re-enable this later once basic communication works
    // if (!spi_hal_wait_for_int(bno)) {
    //     return 0;
    // }

    gpio_put(bno->cs_pin, 0);
    int ret = spi_read_blocking(bno->spi_port, 0x00, buf, len);
    gpio_put(bno->cs_pin, 1);
    
    if (t_us) *t_us = time_us_32();
    return ret;
}

static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    Pico_BNO08x_t *bno = container_of(self, Pico_BNO08x_t, hal);
    
    // For now, let's not wait for INT during initialization to see if that helps
    // if (!spi_hal_wait_for_int(bno)) {
    //     return 0;
    // }

    gpio_put(bno->cs_pin, 0);
    int ret = spi_write_blocking(bno->spi_port, buf, len);
    gpio_put(bno->cs_pin, 1);
    
    return ret;
}

static void hardware_reset(Pico_BNO08x_t *bno) {
    printf("[DEBUG] IMU%d: Hardware reset starting\n", bno->instance_id);
    
    gpio_init(bno->reset_pin);
    gpio_set_dir(bno->reset_pin, GPIO_OUT);
    
    // Perform reset sequence
    gpio_put(bno->reset_pin, 0);
    printf("[DEBUG] IMU%d: Reset pin LOW\n", bno->instance_id);
    sleep_ms(RESET_DELAY_MS);
    
    gpio_put(bno->reset_pin, 1);
    printf("[DEBUG] IMU%d: Reset pin HIGH\n", bno->instance_id);
    sleep_ms(100);
    
    printf("[DEBUG] IMU%d: Hardware reset completed\n", bno->instance_id);
}