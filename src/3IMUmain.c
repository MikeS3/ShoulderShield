#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Pico_BNO08x.h"

// Shared SPI Pins
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
bool imu1_ok = false, imu2_ok = false, imu3_ok = false;

void configure_gpio() {
    gpio_init(CS1_PIN); gpio_set_dir(CS1_PIN, GPIO_OUT);
    gpio_init(CS2_PIN); gpio_set_dir(CS2_PIN, GPIO_OUT);
    gpio_init(CS3_PIN); gpio_set_dir(CS3_PIN, GPIO_OUT);

    gpio_init(RESET1_PIN); gpio_set_dir(RESET1_PIN, GPIO_OUT);
    gpio_init(RESET2_PIN); gpio_set_dir(RESET2_PIN, GPIO_OUT);
    gpio_init(RESET3_PIN); gpio_set_dir(RESET3_PIN, GPIO_OUT);

    gpio_init(INT1_PIN); gpio_set_dir(INT1_PIN, GPIO_IN); gpio_pull_up(INT1_PIN);
    gpio_init(INT2_PIN); gpio_set_dir(INT2_PIN, GPIO_IN); gpio_pull_up(INT2_PIN);
    gpio_init(INT3_PIN); gpio_set_dir(INT3_PIN, GPIO_IN); gpio_pull_up(INT3_PIN);
}

bool init_imu(Pico_BNO08x_t *imu, int reset_pin, int cs_pin, int int_pin) {
    // Reset other IMUs
    if (reset_pin != RESET1_PIN) gpio_put(RESET1_PIN, 0);
    if (reset_pin != RESET2_PIN) gpio_put(RESET2_PIN, 0);
    if (reset_pin != RESET3_PIN) gpio_put(RESET3_PIN, 0);

    // Reset this IMU
    gpio_put(reset_pin, 0);
    sleep_ms(10);
    gpio_put(reset_pin, 1);
    sleep_ms(50);

    if (!pico_bno08x_init(imu, reset_pin, instance_id)) return false;
    return pico_bno08x_begin_spi(imu, SPI_PORT, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN, cs_pin, int_pin, 1000000); //1 MHz Spi
}

void enable_reports(Pico_BNO08x_t *imu) {
    pico_bno08x_enable_report(imu, SH2_ROTATION_VECTOR, 100000);
    pico_bno08x_enable_report(imu, SH2_ACCELEROMETER, 100000);
    pico_bno08x_enable_report(imu, SH2_GYROSCOPE_CALIBRATED, 100000);
    pico_bno08x_enable_report(imu, SH2_MAGNETIC_FIELD_CALIBRATED, 100000);
}
//100,000us = 10Hz refresh rate

void print_sensor(Pico_BNO08x_t *imu, int id) {
    sh2_SensorValue_t val;
    if (pico_bno08x_get_sensor_event(imu, &val)) {
        printf("[IMU%d] ID: %d ", id, val.sensorId);
        if (val.sensorId == SH2_ROTATION_VECTOR) {
            printf("Quat: i=%.2f j=%.2f k=%.2f r=%.2f\n", 
                val.un.rotationVector.i,
                val.un.rotationVector.j,
                val.un.rotationVector.k,
                val.un.rotationVector.real);
        } else if (val.sensorId == SH2_ACCELEROMETER) {
            printf("Accel: x=%.2f y=%.2f z=%.2f\n",
                val.un.accelerometer.x,
                val.un.accelerometer.y,
                val.un.accelerometer.z);
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("Triple IMU SPI Example Start\n");

    configure_gpio();
    //spi set to 1000000 = 1MHz

    imu1_ok = init_imu(&imu1, RESET1_PIN, CS1_PIN, INT1_PIN);
    printf("IMU1 %s\n", imu1_ok ? "initialized" : "failed SPI init");

    imu2_ok = init_imu(&imu2, RESET2_PIN, CS2_PIN, INT2_PIN);
    printf("IMU2 %s\n", imu2_ok ? "initialized" : "failed SPI init");

    imu3_ok = init_imu(&imu3, RESET3_PIN, CS3_PIN, INT3_PIN);
    printf("IMU3 %s\n", imu3_ok ? "initialized" : "failed SPI init");

    if (imu1_ok) enable_reports(&imu1);
    if (imu2_ok) enable_reports(&imu2);
    if (imu3_ok) enable_reports(&imu3);

    while (1) {
        if (imu1_ok) { pico_bno08x_service(&imu1); print_sensor(&imu1, 1); }
        if (imu2_ok) { pico_bno08x_service(&imu2); print_sensor(&imu2, 2); }
        if (imu3_ok) { pico_bno08x_service(&imu3); print_sensor(&imu3, 3); }
        sleep_ms(50);
    }

    return 0;
}
