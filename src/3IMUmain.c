#include <stdio.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

// Shared SPI Bus Configuration
#define SPI_PORT        spi0
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU1 Configuration
#define CS1_PIN         17
#define INT1_PIN        20
#define RESET1_PIN      15

// IMU2 Configuration  
#define CS2_PIN         13
#define INT2_PIN        12
#define RESET2_PIN      14

// IMU3 Configuration
#define CS3_PIN         11
#define INT3_PIN        10
#define RESET3_PIN      9

// Global IMU instances
Pico_BNO08x_t imu1, imu2, imu3;
bool imu1_ok = false, imu2_ok = false, imu3_ok = false;

/**
 * @brief Ensure all other IMUs are held in reset during initialization
 */
void reset_all_other_imus(int exclude_reset_pin) {
    // Put all IMUs in reset except the one we're initializing
    if (exclude_reset_pin != RESET1_PIN) {
        gpio_init(RESET1_PIN);
        gpio_set_dir(RESET1_PIN, GPIO_OUT);
        gpio_put(RESET1_PIN, 0);  // Hold in reset
    }
    if (exclude_reset_pin != RESET2_PIN) {
        gpio_init(RESET2_PIN);
        gpio_set_dir(RESET2_PIN, GPIO_OUT);
        gpio_put(RESET2_PIN, 0);  // Hold in reset
    }
    if (exclude_reset_pin != RESET3_PIN) {
        gpio_init(RESET3_PIN);
        gpio_set_dir(RESET3_PIN, GPIO_OUT);
        gpio_put(RESET3_PIN, 0);  // Hold in reset
    }
    sleep_ms(50);  // Ensure they're properly in reset
}

/**
 * @brief Setup and initialize a single IMU with better isolation
 */
bool setup_imu_isolated(Pico_BNO08x_t *imu, int reset_pin, int instance_id,
                        uint8_t cs_pin, uint8_t int_pin, const char *label) {
    printf("\n[INIT] === Setting up %s (ID: %d) ===\n", label, instance_id);
    
    // Step 1: Hold all other IMUs in reset to prevent SPI interference
    printf("[INIT] %s: Putting other IMUs in reset...\n", label);
    reset_all_other_imus(reset_pin);
    
    // Step 2: Initialize the IMU structure
    printf("[INIT] %s: Initializing IMU structure...\n", label);
    if (!pico_bno08x_init(imu, reset_pin, instance_id)) {
        printf("[ERROR] %s: pico_bno08x_init failed\n", label);
        return false;
    }
    
    // Step 3: Setup CS and INT pins early
    printf("[INIT] %s: Setting up GPIO pins...\n", label);
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);  // CS high (inactive)
    
    gpio_init(int_pin);
    gpio_set_dir(int_pin, GPIO_IN);
    gpio_pull_up(int_pin);
    
    // Step 4: Give extra time before starting SPI communication
    sleep_ms(100);
    
    // Step 5: Start SPI communication with retries
    printf("[INIT] %s: Starting SPI communication...\n", label);
    bool spi_success = false;
    for (int retry = 0; retry < 3; retry++) {
        if (retry > 0) {
            printf("[RETRY] %s: SPI attempt %d/3\n", label, retry + 1);
            sleep_ms(200);
        }
        
        if (pico_bno08x_begin_spi(imu, SPI_PORT,
                                  SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                                  cs_pin, int_pin, 1000000)) {
            spi_success = true;
            break;
        }
    }
    
    if (!spi_success) {
        printf("[ERROR] %s: All SPI attempts failed\n", label);
        return false;
    }
    
    // Step 6: Wait for IMU to be fully ready
    printf("[INIT] %s: Waiting for IMU to be ready...\n", label);
    sleep_ms(300);
    
    // Step 7: Enable sensors with retries
    printf("[INIT] %s: Enabling sensors...\n", label);
    bool sensor_success = false;
    for (int retry = 0; retry < 3; retry++) {
        if (retry > 0) {
            printf("[RETRY] %s: Sensor enable attempt %d/3\n", label, retry + 1);
            sleep_ms(100);
        }
        
        // Try to enable gyro integrated rotation vector first (most reliable)
        if (pico_bno08x_enable_report(imu, SH2_GYRO_INTEGRATED_RV, 50000)) {
            sensor_success = true;
            printf("[SUCCESS] %s: Gyro Integrated RV enabled\n", label);
            break;
        }
    }
    
    if (!sensor_success) {
        printf("[WARN] %s: Failed to enable gyro integrated RV, trying rotation vector\n", label);
        // Fallback to regular rotation vector
        if (pico_bno08x_enable_report(imu, SH2_ROTATION_VECTOR, 100000)) {
            sensor_success = true;
            printf("[SUCCESS] %s: Rotation Vector enabled as fallback\n", label);
        }
    }
    
    // Step 8: Final verification
    if (sensor_success) {
        printf("[SUCCESS] %s: Fully initialized and configured\n", label);
        return true;
    } else {
        printf("[ERROR] %s: Failed to enable any sensors\n", label);
        return false;
    }
}

/**
 * @brief Read and display data from a single IMU
 */
void read_and_display_imu(Pico_BNO08x_t *imu, const char *label) {
    sh2_SensorValue_t value;
    
    // Service the IMU (important!)
    pico_bno08x_service(imu);
    
    // Try to get sensor data
    if (pico_bno08x_get_sensor_event(imu, &value)) {
        if (value.sensorId == SH2_GYRO_INTEGRATED_RV) {
            printf("%s: GIRV q=(%.3f, %.3f, %.3f, %.3f) angVel=(%.3f, %.3f, %.3f)\n",
                   label,
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
                   label,
                   value.un.rotationVector.real,
                   value.un.rotationVector.i,
                   value.un.rotationVector.j,
                   value.un.rotationVector.k,
                   value.un.rotationVector.accuracy);
        }
        else {
            printf("%s: Sensor ID %d\n", label, value.sensorId);
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(3000);  // Wait for USB console to connect
    
    printf("=== 3-IMU BNO085 Multi-Instance Test (FIXED) ===\n");
    printf("Using Fixed SH2 Library with Better Isolation\n\n");
    
    // Setup shared SPI bus first
    printf("[INIT] Initializing shared SPI bus...\n");
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    printf("[SUCCESS] SPI bus initialized\n");
    
    // Initialize each IMU with complete isolation
    printf("\n=== Initializing IMUs with Isolation ===\n");
    
    // Initialize IMU1 first (keep others in reset)
    imu1_ok = setup_imu_isolated(&imu1, RESET1_PIN, 1, CS1_PIN, INT1_PIN, "IMU1");
    sleep_ms(1000);  // Long delay between IMUs
    
    // Initialize IMU2 (keep others in reset)
    imu2_ok = setup_imu_isolated(&imu2, RESET2_PIN, 2, CS2_PIN, INT2_PIN, "IMU2");
    sleep_ms(1000);  // Long delay between IMUs
    
    // Initialize IMU3 (keep others in reset)
    imu3_ok = setup_imu_isolated(&imu3, RESET3_PIN, 3, CS3_PIN, INT3_PIN, "IMU3");
    sleep_ms(1000);
    
    // Now release all IMUs from reset so they can all operate
    printf("\n[INIT] Releasing all IMUs from reset for normal operation...\n");
    if (imu1_ok) {
        gpio_put(RESET1_PIN, 1);
        sleep_ms(100);
    }
    if (imu2_ok) {
        gpio_put(RESET2_PIN, 1);
        sleep_ms(100);
    }
    if (imu3_ok) {
        gpio_put(RESET3_PIN, 1);
        sleep_ms(100);
    }
    
    printf("\n=== Initialization Summary ===\n");
    printf("IMU1: %s\n", imu1_ok ? "✓ OK" : "✗ FAILED");
    printf("IMU2: %s\n", imu2_ok ? "✓ OK" : "✗ FAILED");
    printf("IMU3: %s\n", imu3_ok ? "✓ OK" : "✗ FAILED");
    
    int working_imus = (imu1_ok ? 1 : 0) + (imu2_ok ? 1 : 0) + (imu3_ok ? 1 : 0);
    printf("Working IMUs: %d/3\n", working_imus);
    
    if (working_imus == 0) {
        printf("\n[ERROR] No IMUs working! Check connections and power.\n");
        return -1;
    }
    
    printf("\n=== Starting Data Collection ===\n");
    printf("Data format: LABEL: TYPE q=(w,x,y,z) [additional_data]\n\n");
    
    // Main data collection loop
    uint32_t last_print = 0;
    uint32_t print_interval = 1000000;  // 1 second
    
    while (true) {
        uint32_t now = time_us_32();
        
        // Read from all working IMUs
        if (imu1_ok) {
            read_and_display_imu(&imu1, "IMU1");
        }
        
        if (imu2_ok) {
            read_and_display_imu(&imu2, "IMU2");
        }
        
        if (imu3_ok) {
            read_and_display_imu(&imu3, "IMU3");
        }
        
        // Print separator periodically
        if (now - last_print > print_interval) {
            printf("--- %lu ms --- (%d IMUs active)\n", now / 1000, working_imus);
            last_print = now;
        }
        
        // Delay to prevent overwhelming the console
        sleep_ms(100);
    }
    
    return 0;
}