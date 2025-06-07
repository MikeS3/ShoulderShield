/**
 * BNO08x Example for Raspberry Pi Pico
 * 
 * This example demonstrates how to use the BNO08x IMU sensor with the Raspberry Pi Pico
 * Supports I2C, SPI, and UART interfaces
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "Pico_BNO08x.h"

// Pin definitions - adjust these to match your wiring
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18
#define SPI_CS_PIN      17
#define SPI_INT_PIN     20
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define RESET_PIN       15

// Interface selection - uncomment the one you want to use
//#define USE_I2C_INTERFACE
#define USE_SPI_INTERFACE
// #define USE_UART_INTERFACE

// Function to print quaternion data
void print_quaternion(sh2_SensorValue_t *value) {
    printf("Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.4f\n",
           value->un.rotationVector.i,
           value->un.rotationVector.j,
           value->un.rotationVector.k,
           value->un.rotationVector.real,
           value->un.rotationVector.accuracy);
}

// Function to print accelerometer data
void print_accelerometer(sh2_SensorValue_t *value) {
    printf("Accel: x=%.4f, y=%.4f, z=%.4f m/s²\n",
           value->un.accelerometer.x,
           value->un.accelerometer.y,
           value->un.accelerometer.z);
}

// Function to print gyroscope data
void print_gyroscope(sh2_SensorValue_t *value) {
    printf("Gyro: x=%.4f, y=%.4f, z=%.4f rad/s\n",
           value->un.gyroscope.x,
           value->un.gyroscope.y,
           value->un.gyroscope.z);
}

// Function to print magnetometer data
void print_magnetometer(sh2_SensorValue_t *value) {
    printf("Mag: x=%.4f, y=%.4f, z=%.4f uT\n",
           value->un.magneticField.x,
           value->un.magneticField.y,
           value->un.magneticField.z);
}

int main() {
    stdio_init_all();
    
    // Wait for USB serial connection (optional)
    sleep_ms(2000);
    
    printf("BNO08x IMU Example for Raspberry Pi Pico\n");
    printf("==========================================\n");
    
    // Initialize BNO08x
    Pico_BNO08x_t bno;
    if (!pico_bno08x_init(&bno, RESET_PIN)) {
        printf("Failed to initialize BNO08x structure\n");
        return -1;
    }
    
    bool init_success = false;
    
#ifdef USE_I2C_INTERFACE
    printf("Initializing I2C interface...\n");
    init_success = pico_bno08x_begin_i2c(&bno, i2c0, I2C_SDA_PIN, I2C_SCL_PIN, 
                                        BNO08x_I2CADDR_DEFAULT, 400000);
#elif defined(USE_SPI_INTERFACE)
    printf("Initializing SPI interface...\n");
    init_success = pico_bno08x_begin_spi(&bno, spi0, SPI_MISO_PIN, SPI_MOSI_PIN, 
                                        SPI_SCK_PIN, SPI_CS_PIN, SPI_INT_PIN, 1000000);
#elif defined(USE_UART_INTERFACE)
    printf("Initializing UART interface...\n");
    init_success = pico_bno08x_begin_uart(&bno, uart1, UART_TX_PIN, UART_RX_PIN, 3000000);
#else
    #error "Please define one of USE_I2C_INTERFACE, USE_SPI_INTERFACE, or USE_UART_INTERFACE"
#endif
    
    if (!init_success) {
        printf("Failed to initialize BNO08x!\n");
        return -1;
    }
    
    printf("BNO08x initialized successfully!\n");
    
    // Print product information
    printf("\nProduct Information:\n");
    for (int i = 0; i < bno.prodIds.numEntries; i++) {
        printf("  Entry %d:\n", i);
        printf("    SW Version: %d.%d.%d\n", 
               bno.prodIds.entry[i].swVersionMajor,
               bno.prodIds.entry[i].swVersionMinor,
               bno.prodIds.entry[i].swVersionPatch);
        printf("    Part Number: %lu\n", bno.prodIds.entry[i].swPartNumber);
        printf("    Build Number: %lu\n", bno.prodIds.entry[i].swBuildNumber);
    }
    
    // Enable sensors
    printf("\nEnabling sensors...\n");
    if (!pico_bno08x_enable_report(&bno, SH2_ROTATION_VECTOR, 100000)) { // 100ms = 10Hz
        printf("Failed to enable rotation vector\n");
    }
    if (!pico_bno08x_enable_report(&bno, SH2_ACCELEROMETER, 100000)) { // 100ms = 10Hz
        printf("Failed to enable accelerometer\n");
    }
    if (!pico_bno08x_enable_report(&bno, SH2_GYROSCOPE_CALIBRATED, 100000)) { // 100ms = 10Hz
        printf("Failed to enable gyroscope\n");
    }
    if (!pico_bno08x_enable_report(&bno, SH2_MAGNETIC_FIELD_CALIBRATED, 100000)) { // 100ms = 10Hz
        printf("Failed to enable magnetometer\n");
    }
    
    printf("Sensors enabled. Starting data acquisition...\n\n");
    
    sh2_SensorValue_t sensor_value;
    uint32_t last_print = 0;
    
    while (1) {
        // Service the sensor hub
        pico_bno08x_service(&bno);
        
        // Check for sensor events
        if (pico_bno08x_get_sensor_event(&bno, &sensor_value)) {
            uint32_t now = time_us_32() / 1000; // Convert to ms
            
            // Print data every 500ms to avoid flooding the output
            if (now - last_print > 500) {
                printf("--- Sensor Data (Status: %d) ---\n", sensor_value.status);
                
                switch (sensor_value.sensorId) {
                    case SH2_ROTATION_VECTOR:
                        print_quaternion(&sensor_value);
                        break;
                    case SH2_ACCELEROMETER:
                        print_accelerometer(&sensor_value);
                        break;
                    case SH2_GYROSCOPE_CALIBRATED:
                        print_gyroscope(&sensor_value);
                        break;
                    case SH2_MAGNETIC_FIELD_CALIBRATED:
                        print_magnetometer(&sensor_value);
                        break;
                    default:
                        printf("Unknown sensor ID: 0x%02X\n", sensor_value.sensorId);
                        break;
                }
                
                last_print = now;
                printf("\n");
            }
        }
        
        // Check for reset
        if (pico_bno08x_was_reset(&bno)) {
            printf("BNO08x was reset!\n");
            // Re-enable sensors after reset
            pico_bno08x_enable_report(&bno, SH2_ROTATION_VECTOR, 100000);
            pico_bno08x_enable_report(&bno, SH2_ACCELEROMETER, 100000);
            pico_bno08x_enable_report(&bno, SH2_GYROSCOPE_CALIBRATED, 100000);
            pico_bno08x_enable_report(&bno, SH2_MAGNETIC_FIELD_CALIBRATED, 100000);
        }
        
        // Small delay to prevent overwhelming the system
        sleep_ms(1);
    }
    
    return 0;
}
