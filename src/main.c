/**
 * Dual-BNO08x Example for Raspberry Pi Pico 2W
 * 
 * This example demonstrates how to use 2 BNO08x IMU sensors with the Raspberry Pi Pico 2W
 * Both IMUs use SPI interface with different CS and INT pins
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Pico_BNO08x.h"

// SPI Pin definitions (shared across both IMUs)
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU 1 pins (Upper Arm IMU)
#define IMU1_CS_PIN     17
#define IMU1_INT_PIN    20
#define IMU1_RESET_PIN  15

// IMU 2 pins (Forearm IMU)
#define IMU2_CS_PIN     21
#define IMU2_INT_PIN    22
#define IMU2_RESET_PIN  26

// SPI speed
#define SPI_SPEED       1000000  // 1MHz

// Function to print quaternion data with IMU ID
void print_quaternion(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.4f\n",
           imu_id,
           value->un.rotationVector.i,
           value->un.rotationVector.j,
           value->un.rotationVector.k,
           value->un.rotationVector.real,
           value->un.rotationVector.accuracy);
}

// Function to print accelerometer data with IMU ID
void print_accelerometer(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Accel: x=%.4f, y=%.4f, z=%.4f m/s²\n",
           imu_id,
           value->un.accelerometer.x,
           value->un.accelerometer.y,
           value->un.accelerometer.z);
}

// Function to print gyroscope data with IMU ID
void print_gyroscope(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Gyro: x=%.4f, y=%.4f, z=%.4f rad/s\n",
           imu_id,
           value->un.gyroscope.x,
           value->un.gyroscope.y,
           value->un.gyroscope.z);
}

// Function to print magnetometer data with IMU ID
void print_magnetometer(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Mag: x=%.4f, y=%.4f, z=%.4f uT\n",
           imu_id,
           value->un.magneticField.x,
           value->un.magneticField.y,
           value->un.magneticField.z);
}

int main() {
    stdio_init_all();
    
    // Wait for USB serial connection
    sleep_ms(2000);
    
    printf("Dual-BNO08x IMU Example for Raspberry Pi Pico 2W\n");
    printf("===============================================\n");
    printf("Initializing 2 BNO08x sensors via SPI...\n\n");
    
    // Initialize multi-IMU manager
    Multi_BNO08x_t multi_imu;
    if (!multi_bno08x_init(&multi_imu)) {
        printf("Failed to initialize multi-IMU manager\n");
        return -1;
    }
    
    // Add IMU 1 (Upper Arm)
    printf("Initializing IMU 1 (Upper Arm)...\n");
    if (!multi_bno08x_add_spi_imu(&multi_imu, 0, spi0,
                                 SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                                 IMU1_CS_PIN, IMU1_INT_PIN, IMU1_RESET_PIN, SPI_SPEED)) {
        printf("Failed to initialize IMU 1\n");
    } else {
        printf("IMU 1 initialized successfully\n");
    }
    
    // Add IMU 2 (Forearm)
    printf("Initializing IMU 2 (Forearm)...\n");
    if (!multi_bno08x_add_spi_imu(&multi_imu, 1, spi0,
                                 SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                                 IMU2_CS_PIN, IMU2_INT_PIN, IMU2_RESET_PIN, SPI_SPEED)) {
        printf("Failed to initialize IMU 2\n");
    } else {
        printf("IMU 2 initialized successfully\n");
    }
    
    // Wait a bit for initialization to complete
    sleep_ms(500);
    
    printf("\nActive IMUs: %d\n", multi_bno08x_get_imu_count(&multi_imu));
    
    // Enable sensors on all IMUs
    printf("Enabling rotation vector sensor (10Hz)...\n");
    if (!multi_bno08x_enable_all_reports(&multi_imu, SH2_ROTATION_VECTOR, 100000)) {
        printf("Failed to enable rotation vector on some IMUs\n");
    }
    
    printf("Enabling accelerometer sensor (10Hz)...\n");
    if (!multi_bno08x_enable_all_reports(&multi_imu, SH2_ACCELEROMETER, 100000)) {
        printf("Failed to enable accelerometer on some IMUs\n");
    }
    
    printf("Enabling gyroscope sensor (10Hz)...\n");
    if (!multi_bno08x_enable_all_reports(&multi_imu, SH2_GYROSCOPE_CALIBRATED, 100000)) {
        printf("Failed to enable gyroscope on some IMUs\n");
    }
    
    printf("Enabling magnetometer sensor (10Hz)...\n");
    if (!multi_bno08x_enable_all_reports(&multi_imu, SH2_MAGNETIC_FIELD_CALIBRATED, 100000)) {
        printf("Failed to enable magnetometer on some IMUs\n");
    }
    
    printf("\nStarting data acquisition...\n");
    printf("============================\n\n");
    
    sh2_SensorValue_t sensor_value;
    uint32_t last_print_time = 0;
    uint32_t sensor_count = 0;
    
    while (1) {
        // Service all IMUs
        multi_bno08x_service_all(&multi_imu);
        
        // Check for sensor events from each IMU
        for (uint8_t imu_idx = 0; imu_idx < 2; imu_idx++) {
            if (!multi_bno08x_is_imu_initialized(&multi_imu, imu_idx)) {
                continue;
            }
            
            if (multi_bno08x_get_sensor_event(&multi_imu, imu_idx, &sensor_value)) {
                uint32_t now = time_us_32() / 1000; // Convert to ms
                
                // Print data every 1000ms to avoid flooding the output
                if (now - last_print_time > 1000) {
                    printf("=== IMU %d Data (Status: %d, Count: %lu) ===\n", 
                           imu_idx + 1, sensor_value.status, ++sensor_count);
                    
                    switch (sensor_value.sensorId) {
                        case SH2_ROTATION_VECTOR:
                            print_quaternion(imu_idx + 1, &sensor_value);
                            break;
                        case SH2_ACCELEROMETER:
                            print_accelerometer(imu_idx + 1, &sensor_value);
                            break;
                        case SH2_GYROSCOPE_CALIBRATED:
                            print_gyroscope(imu_idx + 1, &sensor_value);
                            break;
                        case SH2_MAGNETIC_FIELD_CALIBRATED:
                            print_magnetometer(imu_idx + 1, &sensor_value);
                            break;
                        default:
                            printf("IMU%d Unknown sensor ID: 0x%02X\n", imu_idx + 1, sensor_value.sensorId);
                            break;
                    }
                    
                    if (imu_idx == 1) { // After processing IMU 2
                        last_print_time = now;
                        printf("\n");
                    }
                }
            }
            
            // Check for reset events
            if (multi_bno08x_is_imu_initialized(&multi_imu, imu_idx)) {
                // The reset checking would need to be implemented in the multi-IMU interface
                // For now, we'll skip this functionality
            }
        }
        
        // Small delay to prevent overwhelming the system
        sleep_ms(1);
    }
    
    return 0;
}