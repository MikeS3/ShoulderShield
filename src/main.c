// File: main.c — Dual‑IMU with Recommended Fixes

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "Pico_BNO08x.h"

#define SPI_MISO   16
#define SPI_MOSI   19
#define SPI_SCK    18

// IMU1 pins
#define CS1_PIN    17
#define INT1_PIN   20
#define RST1_PIN   15

// IMU2 pins
#define CS2_PIN    13
#define INT2_PIN   12
#define RST2_PIN   14

void print_quat(const char *label, sh2_SensorValue_t *v) {
    printf("%s: i=%.4f j=%.4f k=%.4f r=%.4f acc=%.4f\n",
           label,
           v->un.rotationVector.i,
           v->un.rotationVector.j,
           v->un.rotationVector.k,
           v->un.rotationVector.real,
           v->un.rotationVector.accuracy);
}

void reset_and_wait(uint8_t rst_pin, uint8_t int_pin, const char *label) {
    // Hardware reset
    gpio_put(rst_pin, 0);
    sleep_ms(10);
    gpio_put(rst_pin, 1);

    // Wait ~100 ms for BNO08x internal startup
    sleep_ms(100);

    // Poll INT going low
    printf("%s waiting for INT to go low...", label);
    for (int i = 0; i < 200; i++) {
        if (gpio_get(int_pin) == 0) {
            printf(" done after %dms\n", i);
            return;
        }
        sleep_ms(1);
    }
    printf(" timed out!\n");
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Dual‑BNO08x Stabilized Read\n");
    printf("===========================\n");

    // 1) SPI setup: mode 3, 2 MHz
    spi_init(spi0, 2000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK,  GPIO_FUNC_SPI);

    // 2) INT lines: inputs w/ pull-ups
    gpio_init(INT1_PIN); gpio_set_dir(INT1_PIN, GPIO_IN); gpio_pull_up(INT1_PIN);
    gpio_init(INT2_PIN); gpio_set_dir(INT2_PIN, GPIO_IN); gpio_pull_up(INT2_PIN);

    // 3) CS lines: outputs, idle high
    gpio_init(CS1_PIN); gpio_set_dir(CS1_PIN, GPIO_OUT); gpio_put(CS1_PIN, 1);
    gpio_init(CS2_PIN); gpio_set_dir(CS2_PIN, GPIO_OUT); gpio_put(CS2_PIN, 1);

    // 4) Prepare IMU structs
    Pico_BNO08x_t imu1, imu2;
    pico_bno08x_init(&imu1, RST1_PIN, 0);
    pico_bno08x_init(&imu2, RST2_PIN, 1);

    // 5) Sequential Reset + Init IMU1
    reset_and_wait(RST1_PIN, INT1_PIN, "IMU1");
    printf("IMU1 begin_spi => %d\n",
        pico_bno08x_begin_spi(&imu1, spi0,
                              SPI_MISO, SPI_MOSI, SPI_SCK,
                              CS1_PIN, INT1_PIN, 2000000));
    pico_bno08x_enable_report(&imu1, SH2_ROTATION_VECTOR, 100000);

    // small gap before IMU2
    sleep_ms(200);

    // 6) Sequential Reset + Init IMU2
    reset_and_wait(RST2_PIN, INT2_PIN, "IMU2");
    printf("IMU2 begin_spi => %d\n",
        pico_bno08x_begin_spi(&imu2, spi0,
                              SPI_MISO, SPI_MOSI, SPI_SCK,
                              CS2_PIN, INT2_PIN, 2000000));
    pico_bno08x_enable_report(&imu2, SH2_ROTATION_VECTOR, 100000);

    // 7) Main loop: poll INT, service only that IMU
    sh2_SensorValue_t v;
    while (1) {
        // IMU1 if interrupt
        if (gpio_get(INT1_PIN) == 0) {
            gpio_put(CS2_PIN, 1);       // ensure IMU2 is deselected
            gpio_put(CS1_PIN, 0);
            sleep_us(5);
            pico_bno08x_service(&imu1);
            if (pico_bno08x_get_sensor_event(&imu1, &v)) {
                print_quat("IMU1", &v);
            }
            gpio_put(CS1_PIN, 1);
        }

        // IMU2 if interrupt
        if (gpio_get(INT2_PIN) == 0) {
            gpio_put(CS1_PIN, 1);
            gpio_put(CS2_PIN, 0);
            sleep_us(5);
            pico_bno08x_service(&imu2);
            if (pico_bno08x_get_sensor_event(&imu2, &v)) {
                print_quat("IMU2", &v);
            }
            gpio_put(CS2_PIN, 1);
        }

        sleep_ms(1);
    }
    return 0;
}
