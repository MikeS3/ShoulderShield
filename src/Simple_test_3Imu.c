#include <stdio.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

// SPI Pins
#define SPI_PORT        spi0
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU1 Pins
#define CS1_PIN         17
#define INT1_PIN        20
#define RESET1_PIN      15

// IMU2 Pins
#define CS2_PIN         13
#define INT2_PIN        12
#define RESET2_PIN      14

// IMU3 Pins
#define CS3_PIN         11
#define INT3_PIN        10
#define RESET3_PIN      9

Pico_BNO08x_t imu1, imu2, imu3;

void setup_imu(Pico_BNO08x_t *imu, int reset, int instance,
               uint8_t cs, uint8_t int_pin,
               const char *label) {
    if (!pico_bno08x_init(imu, reset, instance)) {
        printf("[ERROR] %s init failed!\n", label);
        return;
    }

    if (!pico_bno08x_begin_spi(imu, SPI_PORT, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                               cs, int_pin, 1000000)) {
        printf("[ERROR] %s SPI begin failed!\n", label);
        return;
    }

    if (!pico_bno08x_enable_report(imu, SH2_GYRO_INTEGRATED_RV, 10000)) {
        printf("[ERROR] %s report enable failed!\n", label);
        return;
    }

    printf("[OK] %s initialized successfully\n", label);
}

int main() {
    stdio_init_all();
    sleep_ms(3000);  // wait for USB console

    printf("Initializing 3 BNO08x IMUs...\n");

    setup_imu(&imu1, RESET1_PIN, 1, CS1_PIN, INT1_PIN, "IMU1");
    setup_imu(&imu2, RESET2_PIN, 2, CS2_PIN, INT2_PIN, "IMU2");
    setup_imu(&imu3, RESET3_PIN, 3, CS3_PIN, INT3_PIN, "IMU3");

    while (true) {
        sh2_SensorValue_t val;

        if (pico_bno08x_get_sensor_event(&imu1, &val)) {
            printf("IMU1: q=(%.2f, %.2f, %.2f, %.2f)\n", val.un.gyroIntegratedRV.real,
                   val.un.gyroIntegratedRV.i, val.un.gyroIntegratedRV.j, val.un.gyroIntegratedRV.k);
        }

        if (pico_bno08x_get_sensor_event(&imu2, &val)) {
            printf("IMU2: q=(%.2f, %.2f, %.2f, %.2f)\n", val.un.gyroIntegratedRV.real,
                   val.un.gyroIntegratedRV.i, val.un.gyroIntegratedRV.j, val.un.gyroIntegratedRV.k);
        }

        if (pico_bno08x_get_sensor_event(&imu3, &val)) {
            printf("IMU3: q=(%.2f, %.2f, %.2f, %.2f)\n", val.un.gyroIntegratedRV.real,
                   val.un.gyroIntegratedRV.i, val.un.gyroIntegratedRV.j, val.un.gyroIntegratedRV.k);
        }

        sleep_ms(10);  // Reduce CPU usage
    }

    return 0;
}
