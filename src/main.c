/*
 * Triple-BNO08x Example for Raspberry Pi Pico 2W (Supports 3 IMUs)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Pico_BNO08x.h"

// SPI shared pins
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU 1 (Upper Arm)
#define IMU1_CS_PIN     17
#define IMU1_INT_PIN    20
#define IMU1_RESET_PIN  15

// IMU 2 (Forearm)
#define IMU2_CS_PIN     21
#define IMU2_INT_PIN    22
#define IMU2_RESET_PIN  26

// IMU 3 (Torso)
#define IMU3_CS_PIN     27
#define IMU3_INT_PIN    28
#define IMU3_RESET_PIN  14

#define SPI_SPEED       1000000  // 1MHz

// Print helpers
void print_quaternion(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.4f\n",
           imu_id,
           value->un.rotationVector.i,
           value->un.rotationVector.j,
           value->un.rotationVector.k,
           value->un.rotationVector.real,
           value->un.rotationVector.accuracy);
}

void print_accelerometer(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Accel: x=%.4f, y=%.4f, z=%.4f m/s²\n",
           imu_id,
           value->un.accelerometer.x,
           value->un.accelerometer.y,
           value->un.accelerometer.z);
}

void print_gyroscope(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Gyro: x=%.4f, y=%.4f, z=%.4f rad/s\n",
           imu_id,
           value->un.gyroscope.x,
           value->un.gyroscope.y,
           value->un.gyroscope.z);
}

void print_magnetometer(uint8_t imu_id, sh2_SensorValue_t *value) {
    printf("IMU%d Mag: x=%.4f, y=%.4f, z=%.4f uT\n",
           imu_id,
           value->un.magneticField.x,
           value->un.magneticField.y,
           value->un.magneticField.z);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Triple-BNO08x IMU Example for Raspberry Pi Pico 2W\n");
    printf("==================================================\n");

    Multi_BNO08x_t multi_imu;
    multi_bno08x_init(&multi_imu);

    // Initialize all 3 IMUs
    multi_bno08x_add_spi_imu(&multi_imu, 0, spi0, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                             IMU1_CS_PIN, IMU1_INT_PIN, IMU1_RESET_PIN, SPI_SPEED);

    multi_bno08x_add_spi_imu(&multi_imu, 1, spi0, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                             IMU2_CS_PIN, IMU2_INT_PIN, IMU2_RESET_PIN, SPI_SPEED);

    multi_bno08x_add_spi_imu(&multi_imu, 2, spi0, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                             IMU3_CS_PIN, IMU3_INT_PIN, IMU3_RESET_PIN, SPI_SPEED);

    sleep_ms(500);

    printf("\nActive IMUs: %d\n", multi_bno08x_get_imu_count(&multi_imu));

    // Enable sensors on all IMUs at 10Hz (100000 us)
    multi_bno08x_enable_all_reports(&multi_imu, SH2_ROTATION_VECTOR, 100000);
    multi_bno08x_enable_all_reports(&multi_imu, SH2_ACCELEROMETER, 100000);
    multi_bno08x_enable_all_reports(&multi_imu, SH2_GYROSCOPE_CALIBRATED, 100000);
    multi_bno08x_enable_all_reports(&multi_imu, SH2_MAGNETIC_FIELD_CALIBRATED, 100000);

    printf("\nStarting data acquisition...\n\n");

    sh2_SensorValue_t sensor_value;
    uint32_t last_print_time = 0;
    uint32_t sensor_count = 0;

    while (1) {
        multi_bno08x_service_all(&multi_imu);

        for (uint8_t imu_idx = 0; imu_idx < multi_imu.imu_count; imu_idx++) {
            if (!multi_bno08x_is_imu_initialized(&multi_imu, imu_idx)) continue;

            while (multi_bno08x_get_sensor_event(&multi_imu, imu_idx, &sensor_value)) {
                uint32_t now = time_us_32() / 1000;

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
                    last_print_time = now;
                    printf("\n");
                }
            }
        }
        sleep_ms(1);
    }

    return 0;
}
