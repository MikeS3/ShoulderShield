#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Pico_BNO08x.h"

// SPI0 Pins for IMU1
#define SPI0_PORT        spi0
#define SPI0_MISO_PIN    16
#define SPI0_MOSI_PIN    19
#define SPI0_SCK_PIN     18
#define CS1_PIN          17
#define INT1_PIN         20
#define RESET1_PIN       15

// SPI1 Pins for IMU2
#define SPI1_PORT        spi1
#define SPI1_MISO_PIN    12
#define SPI1_MOSI_PIN    11
#define SPI1_SCK_PIN     10
#define CS2_PIN          13
#define INT2_PIN         9
#define RESET2_PIN       14

Pico_BNO08x_t imu1;
Pico_BNO08x_t imu2;

void print_quat(const char* label, sh2_SensorValue_t* value) {
    if (value->sensorId == SH2_ROTATION_VECTOR) {
        printf("%s: quat = [%.4f, %.4f, %.4f, %.4f]\n",
            label,
            value->un.rotationVector.real,
            value->un.rotationVector.i,
            value->un.rotationVector.j,
            value->un.rotationVector.k);
    }
}

int main() {
    stdio_init_all();
    //sleep_ms(500);
    printf("Starting dual-IMU SPI example...\n");

    // Init IMU1 on SPI0
    if (!pico_bno08x_begin_spi(&imu1, SPI0_PORT,
                               SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_SCK_PIN,
                               CS1_PIN, INT1_PIN, 3000000)) {
        printf("Failed to initialize IMU1 on SPI0\n");
    } else {
        printf("IMU1 initialized on SPI0\n");
        pico_bno08x_enable_report(&imu1, SH2_ROTATION_VECTOR, 5000);  // 200Hz
    }

    // Init IMU2 on SPI1
    if (!pico_bno08x_begin_spi(&imu2, SPI1_PORT,
                               SPI1_MISO_PIN, SPI1_MOSI_PIN, SPI1_SCK_PIN,
                               CS2_PIN, INT2_PIN, 3000000)) {
        printf("Failed to initialize IMU2 on SPI1\n");
    } else {
        printf("IMU2 initialized on SPI1\n");
        pico_bno08x_enable_report(&imu2, SH2_ROTATION_VECTOR, 5000);
    }

    while (true) {
        sh2_SensorValue_t value;

        if (pico_bno08x_get_sensor_event(&imu1, &value)) {
            print_quat("IMU1", &value);
        }

        if (pico_bno08x_get_sensor_event(&imu2, &value)) {
            print_quat("IMU2", &value);
        }

        sleep_ms(10);
    }

    return 0;
}
