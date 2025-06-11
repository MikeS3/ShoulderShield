// File: main.c — Solo IMU2 Diagnostic Test

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "Pico_BNO08x.h"

#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU2 pins only
#define SPI_CS_PIN      13
#define SPI_INT_PIN     12
#define RESET_PIN       14

void print_quaternion(sh2_SensorValue_t *value) {
    printf("Quaternion: i=%.4f, j=%.4f, k=%.4f, real=%.4f, accuracy=%.4f\n",
           value->un.rotationVector.i,
           value->un.rotationVector.j,
           value->un.rotationVector.k,
           value->un.rotationVector.real,
           value->un.rotationVector.accuracy);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Solo-IMU2 Diagnostic Test\n");
    printf("=========================\n");

    // 1) Check INT pin wiring/pull-up
    gpio_init(SPI_INT_PIN);
    gpio_set_dir(SPI_INT_PIN, GPIO_IN);
    gpio_pull_up(SPI_INT_PIN);
    int int_level = gpio_get(SPI_INT_PIN);
    printf("INT2 pin reads %d (should be 1 idle-high)\n", int_level);

    // 2) Watch for INT going low after reset
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    printf("Performing hardware reset on pin %d...\n", RESET_PIN);
    gpio_put(RESET_PIN, 0);
    sleep_ms(10);
    gpio_put(RESET_PIN, 1);

    printf("Waiting for INT2 to go low (boot complete)...\n");
    for (int i = 0; i < 200; i++) {
        if (gpio_get(SPI_INT_PIN) == 0) {
            printf("  INT2 went low after %d ms\n", i);
            break;
        }
        sleep_ms(1);
    }

    // 3) Initialize SPI bus
    spi_init(spi0, 1000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN,  GPIO_FUNC_SPI);

    // 4) Initialize and reset IMU2 struct
    Pico_BNO08x_t imu2;
    if (!pico_bno08x_init(&imu2, RESET_PIN, 1)) {
        printf("IMU2: struct init failed\n");
        return -1;
    }

    // 5) Chip-select debug
    gpio_init(SPI_CS_PIN);
    gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
    gpio_put(SPI_CS_PIN, 1);
    printf("CS2 pin is GPIO %d\n", SPI_CS_PIN);

    // 6) Now attempt SPI + SH2 initialization
    printf("IMU2: calling begin_spi...\n");
    bool ok = pico_bno08x_begin_spi(&imu2, spi0,
                                    SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                                    SPI_CS_PIN, SPI_INT_PIN, 1000000);
    printf("IMU2: begin_spi returned %d\n", ok);
    if (!ok) {
        printf("IMU2: SPI init failed—check CS/INT wiring and pull-ups\n");
        return -1;
    }
    printf("IMU2: SPI init OK\n");

    // 7) Enable rotation vector
    if (!pico_bno08x_enable_report(&imu2, SH2_ROTATION_VECTOR, 100000)) {
        printf("IMU2: enable rotation failed\n");
        return -1;
    }
    printf("IMU2: rotation vector enabled\n");

    // 8) Main loop: print data if available
    sh2_SensorValue_t val;
    uint32_t last = 0;
    while (1) {
        pico_bno08x_service(&imu2);
        if (pico_bno08x_get_sensor_event(&imu2, &val)) {
            uint32_t now = time_us_32() / 1000;
            if (now - last > 500) {
                print_quaternion(&val);
                last = now;
            }
        }
        sleep_ms(1);
    }

    return 0;
}
