#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Pico_BNO08x.h"

// IMU1 on SPI0
#define CS1_PIN    17
#define INT1_PIN   20
#define RESET1_PIN 15

// IMU2 on SPI1
#define CS2_PIN    13
#define INT2_PIN   12
#define RESET2_PIN 14

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("Dual‑IMU SPI example\n");

    Pico_BNO08x_t imu1, imu2;
    sh2_SensorValue_t evt;

    // Init IMU1 (SPI0 pins 16/19/18)
    pico_bno08x_init(&imu1, RESET1_PIN);
    pico_bno08x_begin_spi(&imu1, spi0, 16, 19, 18, CS1_PIN, INT1_PIN, 1000000);
    pico_bno08x_set_active(&imu1);
    pico_bno08x_enable_report(&imu1, SH2_ROTATION_VECTOR, 10000);

    // Init IMU2 (SPI1 pins 12/11/10)
    pico_bno08x_init(&imu2, RESET2_PIN);
    pico_bno08x_begin_spi(&imu2, spi1, 12, 11, 10, CS2_PIN, INT2_PIN, 1000000);
    pico_bno08x_set_active(&imu2);
    pico_bno08x_enable_report(&imu2, SH2_ROTATION_VECTOR, 10000);

    while (1) {
        // IMU1
        pico_bno08x_set_active(&imu1);
        if (pico_bno08x_get_sensor_event(&imu1, &evt)) {
            printf("IMU1 quat: [%.4f,%.4f,%.4f,%.4f]\n",
                   evt.un.rotationVector.real,
                   evt.un.rotationVector.i,
                   evt.un.rotationVector.j,
                   evt.un.rotationVector.k);
        }

        // IMU2
        pico_bno08x_set_active(&imu2);
        if (pico_bno08x_get_sensor_event(&imu2, &evt)) {
            printf("IMU2 quat: [%.4f,%.4f,%.4f,%.4f]\n",
                   evt.un.rotationVector.real,
                   evt.un.rotationVector.i,
                   evt.un.rotationVector.j,
                   evt.un.rotationVector.k);
        }

        sleep_ms(10);
    }
    return 0;
}
