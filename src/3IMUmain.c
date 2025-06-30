#include <stdio.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

// SPI Configuration
#define SPI_PORT        spi0
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU Configurations
typedef struct {
    uint8_t cs_pin;
    uint8_t int_pin;
    uint8_t reset_pin;
    const char* name;
} imu_config_t;

static const imu_config_t imu_configs[3] = {
    {.cs_pin = 17, .int_pin = 20, .reset_pin = 15, .name = "IMU1"},
    {.cs_pin = 13, .int_pin = 12, .reset_pin = 14, .name = "IMU2"}, 
    {.cs_pin = 11, .int_pin = 10, .reset_pin = 9,  .name = "IMU3"}
};

// Single IMU instance that gets reused
Pico_BNO08x_t active_imu;
int current_imu = -1;
bool imu_available[3] = {false, false, false};

/**
 * @brief Initialize all IMU hardware without SH2 layer
 */
void init_imu_hardware() {
    printf("[INIT] Setting up SPI bus and IMU hardware...\n");
    
    // Setup SPI bus
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    
    // Setup all IMU pins
    for (int i = 0; i < 3; i++) {
        // CS pins (start high/inactive)
        gpio_init(imu_configs[i].cs_pin);
        gpio_set_dir(imu_configs[i].cs_pin, GPIO_OUT);
        gpio_put(imu_configs[i].cs_pin, 1);
        
        // INT pins  
        gpio_init(imu_configs[i].int_pin);
        gpio_set_dir(imu_configs[i].int_pin, GPIO_IN);
        gpio_pull_up(imu_configs[i].int_pin);
        
        // Reset pins (start in reset)
        gpio_init(imu_configs[i].reset_pin);
        gpio_set_dir(imu_configs[i].reset_pin, GPIO_OUT);
        gpio_put(imu_configs[i].reset_pin, 0);
    }
    
    sleep_ms(100);
    
    // Release all from reset
    for (int i = 0; i < 3; i++) {
        gpio_put(imu_configs[i].reset_pin, 1);
        sleep_ms(50);
    }
    
    sleep_ms(200);  // Let all IMUs boot
    
    printf("[SUCCESS] IMU hardware initialized\n");
}

/**
 * @brief Test if an IMU is responding on SPI
 */
bool test_imu_communication(int imu_id) {
    if (imu_id < 0 || imu_id >= 3) return false;
    
    const imu_config_t* config = &imu_configs[imu_id];
    
    // Simple SPI test - try to read some bytes
    uint8_t test_data[4] = {0};
    
    gpio_put(config->cs_pin, 0);
    sleep_us(10);
    
    int result = spi_read_blocking(SPI_PORT, 0x00, test_data, 4);
    
    sleep_us(10);
    gpio_put(config->cs_pin, 1);
    
    // Check if we got non-zero data (indicates IMU is responding)
    bool responding = false;
    for (int i = 0; i < 4; i++) {
        if (test_data[i] != 0x00 && test_data[i] != 0xFF) {
            responding = true;
            break;
        }
    }
    
    printf("[TEST] %s SPI test: %s (data: %02X %02X %02X %02X)\n", 
           config->name, responding ? "PASS" : "FAIL",
           test_data[0], test_data[1], test_data[2], test_data[3]);
    
    return responding;
}

/**
 * @brief Switch to a specific IMU (time-division multiplexing)
 */
bool switch_to_imu(int imu_id) {
    if (imu_id < 0 || imu_id >= 3) return false;
    if (!imu_available[imu_id]) return false;
    if (current_imu == imu_id) return true;  // Already active
    
    // Close current IMU if any
    if (current_imu >= 0) {
        printf("[SWITCH] Closing %s\n", imu_configs[current_imu].name);
        pico_bno08x_destroy(&active_imu);
    }
    
    // Initialize new IMU
    printf("[SWITCH] Opening %s\n", imu_configs[imu_id].name);
    
    if (!pico_bno08x_init(&active_imu, imu_configs[imu_id].reset_pin, imu_id)) {
        printf("[ERROR] Failed to init %s\n", imu_configs[imu_id].name);
        current_imu = -1;
        return false;
    }
    
    if (!pico_bno08x_begin_spi(&active_imu, SPI_PORT,
                               SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                               imu_configs[imu_id].cs_pin, 
                               imu_configs[imu_id].int_pin, 
                               1000000)) {
        printf("[ERROR] Failed to start SPI for %s\n", imu_configs[imu_id].name);
        pico_bno08x_destroy(&active_imu);
        current_imu = -1;
        return false;
    }
    
    // Enable sensors
    sleep_ms(100);
    pico_bno08x_enable_report(&active_imu, SH2_GYRO_INTEGRATED_RV, 50000);
    pico_bno08x_enable_report(&active_imu, SH2_ROTATION_VECTOR, 100000);
    
    current_imu = imu_id;
    printf("[SUCCESS] Switched to %s\n", imu_configs[imu_id].name);
    return true;
}

/**
 * @brief Read data from currently active IMU
 */
bool read_current_imu_data(sh2_SensorValue_t* value) {
    if (current_imu < 0) return false;
    
    pico_bno08x_service(&active_imu);
    return pico_bno08x_get_sensor_event(&active_imu, value);
}

int main() {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("=== Time-Division Multi-IMU BNO085 Test ===\n");
    printf("Strategy: One IMU active at a time, cycle through them\n\n");
    
    // Initialize hardware
    init_imu_hardware();
    
    // Test which IMUs are responding
    printf("\n=== Testing IMU Communication ===\n");
    int available_count = 0;
    for (int i = 0; i < 3; i++) {
        if (test_imu_communication(i)) {
            imu_available[i] = true;
            available_count++;
        }
    }
    
    printf("\n=== IMU Availability Summary ===\n");
    for (int i = 0; i < 3; i++) {
        printf("%s: %s\n", imu_configs[i].name, 
               imu_available[i] ? "✓ Available" : "✗ Not responding");
    }
    printf("Total available: %d/3\n", available_count);
    
    if (available_count == 0) {
        printf("\n[ERROR] No IMUs responding! Check connections.\n");
        return -1;
    }
    
    printf("\n=== Starting Time-Division Data Collection ===\n");
    printf("Cycling through available IMUs every 2 seconds...\n\n");
    
    uint32_t last_switch = 0;
    uint32_t switch_interval = 2000000;  // 2 seconds in microseconds
    int next_imu = 0;
    
    while (true) {
        uint32_t now = time_us_32();
        
        // Check if it's time to switch IMUs
        if (now - last_switch > switch_interval) {
            // Find next available IMU
            for (int i = 0; i < 3; i++) {
                int candidate = (next_imu + i) % 3;
                if (imu_available[candidate]) {
                    if (switch_to_imu(candidate)) {
                        next_imu = (candidate + 1) % 3;
                        last_switch = now;
                        break;
                    }
                }
            }
        }
        
        // Read data from current IMU
        if (current_imu >= 0) {
            sh2_SensorValue_t value;
            if (read_current_imu_data(&value)) {
                const char* name = imu_configs[current_imu].name;
                
                if (value.sensorId == SH2_GYRO_INTEGRATED_RV) {
                    printf("%s: GIRV q=(%.3f, %.3f, %.3f, %.3f) ω=(%.3f, %.3f, %.3f)\n",
                           name,
                           value.un.gyroIntegratedRV.real,
                           value.un.gyroIntegratedRV.i,
                           value.un.gyroIntegratedRV.j,
                           value.un.gyroIntegratedRV.k,
                           value.un.gyroIntegratedRV.angVelX,
                           value.un.gyroIntegratedRV.angVelY,
                           value.un.gyroIntegratedRV.angVelZ);
                }
                else if (value.sensorId == SH2_ROTATION_VECTOR) {
                    printf("%s: RV q=(%.3f, %.3f, %.3f, %.3f) acc=%.3f\n",
                           name,
                           value.un.rotationVector.real,
                           value.un.rotationVector.i,
                           value.un.rotationVector.j,
                           value.un.rotationVector.k,
                           value.un.rotationVector.accuracy);
                }
            }
        }
        
        sleep_ms(50);
    }
    
    return 0;
}