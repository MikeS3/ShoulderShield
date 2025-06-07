/**
 * Multi-BNO08x Example for Raspberry Pi Pico
 * 
 * This example demonstrates how to use 3 BNO08x IMU sensors with the Raspberry Pi Pico
 * All IMUs use SPI interface with different CS and INT pins
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Pico_BNO08x.h"

// SPI Pin definitions (shared across all IMUs)
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU 1 pins
#define IMU1_CS_PIN     17
#define IMU1_INT_PIN    20
#define IMU1_RESET_PIN  15

// IMU 2 pins
#define IMU2_CS_PIN     21
#define IMU2_INT_PIN    22
#define IMU2_RESET_PIN  26

// IMU 3 pins
#define IMU3_CS_PIN     27
#define IMU3_INT_PIN    28
#define IMU3_RESET_PIN  29

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
    
    printf("Multi-BNO08x IMU Example for Raspberry Pi Pico\n");
    printf("==============================================\n");
    printf("Initializing 3 BNO08x sensors via SPI...\n\n");
    
    // Initialize multi-IMU manager
    Multi_BNO08x_t multi_imu;
    if (!multi_bno08x_init(&multi_imu)) {
        printf("Failed to initialize multi-IMU manager\n");
        return -1;
    }
    
    // Add IMU 1
    printf("Initializing IMU 1...\n");
    if (!multi_bno08x_add_spi_imu(&multi_imu, 0, spi0,
                                 SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                                 IMU1_CS_PIN, IMU1_INT_PIN, IMU1_RESET_PIN, SPI_SPEED)) {
        printf("Failed to initialize IMU 1\n");
    } else {
        printf("IMU 1 initialized successfully\n");
    }
    
    // Add IMU 2
    printf("Initializing IMU 2...\n");
    if (!multi_bno08x_add_spi_imu(&multi_imu, 1, spi0,
                                 SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                                 IMU2_CS_PIN, IMU2_INT_PIN, IMU2_RESET_PIN, SPI_SPEED)) {
        printf("Failed to initialize IMU 2\n");
    } else {
        printf("IM
