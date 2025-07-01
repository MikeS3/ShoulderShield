#include <stdio.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

#define SPI_PORT        spi0
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU2 pins
#define CS_PIN          9
#define INT_PIN         10
#define RESET_PIN       11

Pico_BNO08x_t imu;

int main() {
    stdio_init_all();
    sleep_ms(2000);

    // --- SPI + pin setup (mode 1, pull-down on INT) ---
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);

    gpio_init(CS_PIN);   gpio_set_dir(CS_PIN, GPIO_OUT);   gpio_put(CS_PIN, 1);
    gpio_init(INT_PIN);  gpio_set_dir(INT_PIN, GPIO_IN);   gpio_pull_down(INT_PIN);
    gpio_init(RESET_PIN);gpio_set_dir(RESET_PIN, GPIO_OUT);gpio_put(RESET_PIN, 0);
    sleep_ms(100);
    gpio_put(RESET_PIN, 1);
    sleep_ms(200);

    // --- Driver init ---
    if (!pico_bno08x_init(&imu, RESET_PIN, 2) ||
        !pico_bno08x_begin_spi(&imu,
                               SPI_PORT, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
                               CS_PIN, INT_PIN, 1000000)) {
        printf("IMU init failed\n");
        return -1;
    }

    // --- Enable quaternion (rotation vector) report at 50 Hz ---
    pico_bno08x_enable_report(&imu, SH2_ROTATION_VECTOR, 20000);
    sleep_ms(100);

    // --- Read & print quaternion forever ---
    while (true) {
        pico_bno08x_service(&imu);

        sh2_SensorValue_t evt;
        if (pico_bno08x_get_sensor_event(&imu, &evt)
            && evt.sensorId == SH2_ROTATION_VECTOR) {

            float w = evt.un.rotationVector.real;
            float x = evt.un.rotationVector.i;
            float y = evt.un.rotationVector.j;
            float z = evt.un.rotationVector.k;

            printf("Q = [w: %.3f, x: %.3f, y: %.3f, z: %.3f]\n",
                   w, x, y, z);
        }
        sleep_ms(20);
    }

    return 0;
}
