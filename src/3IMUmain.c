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
 * @brief Setup and initialize a single IMU
 */
bool setup_imu(Pico_BNO08x_t *imu, int reset_pin, int instance_id,
               uint8_t cs_pin, uint8_t int_pin, const char *label) {
    printf("[INIT] Setting up %s (ID: %d)\n", label, instance_id);
    
    // Initialize the IMU structure
    if (!pico_bno08x_init(imu, reset_pin, instance_id)) {
        printf("[ERROR] %s: pico_bno08x_init failed\n", label);
        return false;
    }
    
    // Start SPI communication
    if (!pico_bno08x_begin_spi(imu, SPI_PORT,
                               SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                               cs_pin, int_pin, 1000000)) {
        printf("[ERROR] %s: pico_bno08x_begin_spi failed\n", label);
        return false;
    }
    
    // Small delay to ensure IMU is ready
    sleep_ms(100);
    
    // Enable gyro integrated rotation vector (good for testing)
    if (!pico_bno08x_enable_report(imu, SH2_GYRO_INTEGRATED_RV, 50000)) {  // 20Hz
        printf("[WARN] %s: Failed to enable gyro integrated RV\n", label);
    }
    
    // Enable regular rotation vector as backup
    if (!pico_bno08x_enable_report(imu, SH2_ROTATION_VECTOR, 100000)) {  // 10Hz
        printf("[WARN] %s: Failed to enable rotation vector\n", label);
    }
    
    printf("[SUCCESS] %s: Initialized and configured\n", label);
    return true;
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
    
    printf("=== 3-IMU BNO085 Multi-Instance Test ===\n");
    printf("Using Fixed SH2 Library with Multiple Instances\n\n");
    
    // Setup shared SPI bus first
    printf("[INIT] Initializing shared SPI bus...\n");
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    printf("[SUCCESS] SPI bus initialized\n\n");
    
    // Initialize each IMU with delays between them
    printf("=== Initializing IMUs ===\n");
    
    imu1_ok = setup_imu(&imu1, RESET1_PIN, 1, CS1_PIN, INT1_PIN, "IMU1");
    sleep_ms(500);  // Delay between IMUs
    
    imu2_ok = setup_imu(&imu2, RESET2_PIN, 2, CS2_PIN, INT2_PIN, "IMU2");
    sleep_ms(500);  // Delay between IMUs
    
    imu3_ok = setup_imu(&imu3, RESET3_PIN, 3, CS3_PIN, INT3_PIN, "IMU3");
    sleep_ms(500);
    
    printf("\n=== Initialization Summary ===\n");
    printf("IMU1: %s\n", imu1_ok ? "OK" : "FAILED");
    printf("IMU2: %s\n", imu2_ok ? "OK" : "FAILED");
    printf("IMU3: %s\n", imu3_ok ? "OK" : "FAILED");
    
    if (!imu1_ok && !imu2_ok && !imu3_ok) {
        printf("\n[ERROR] All IMUs failed to initialize!\n");
        return -1;
    }
    
    printf("\n=== Starting Data Collection ===\n");
    printf("Data format: LABEL: TYPE q=(w,x,y,z) [additional_data]\n\n");
    
    // Main data collection loop
    uint32_t last_print = 0;
    uint32_t print_interval = 500000;  // 500ms
    
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
            printf("--- %lu ms ---\n", now / 1000);
            last_print = now;
        }
        
        // Small delay to prevent overwhelming the console
        sleep_ms(50);
    }
    
    // Cleanup (though we never reach here)
    if (imu1_ok) pico_bno08x_destroy(&imu1);
    if (imu2_ok) pico_bno08x_destroy(&imu2);
    if (imu3_ok) pico_bno08x_destroy(&imu3);
    
    return 0;
}