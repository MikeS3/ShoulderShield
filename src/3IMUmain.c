#include <stdio.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

// SPI Configuration
#define SPI_PORT        spi0
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// Time-division parameters
#define SWITCH_INTERVAL_MS 2000   // switch every 2 seconds
#define POLL_ITERATIONS     10    // poll ~0.5s after switch
#define POLL_DELAY_MS       50

// IMU configurations (CS, INT, RESET pins)
typedef struct {
    uint cs;
    uint inten;
    uint rst;
    const char* name;
} imu_cfg_t;

static const imu_cfg_t imus[3] = {
    {17, 20, 15, "IMU0"},
    {13, 12, 14, "IMU1"},
    {9,  10, 11, "IMU2"}
};

// State
Pico_BNO08x_t active;
int cur = -1;
bool available[3] = { false };

// Initialize SPI & all pins
void init_hw() {
    stdio_init_all();
    sleep_ms(2000);

    // SPI mode 1
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);

    for (int i = 0; i < 3; i++) {
        gpio_init(imus[i].cs);  gpio_set_dir(imus[i].cs, GPIO_OUT);  gpio_put(imus[i].cs, 1);
        gpio_init(imus[i].inten);  gpio_set_dir(imus[i].inten, GPIO_IN);  gpio_pull_down(imus[i].inten);
        gpio_init(imus[i].rst); gpio_set_dir(imus[i].rst, GPIO_OUT); gpio_put(imus[i].rst, 0);
    }
    sleep_ms(100);
    for (int i = 0; i < 3; i++) {
        gpio_put(imus[i].rst, 1);
        sleep_ms(50);
    }
    sleep_ms(200);
}

// Raw SPI ping
bool check_resp(int i) {
    uint8_t buf[4] = {0};
    gpio_put(imus[i].cs, 0); sleep_us(10);
    spi_read_blocking(SPI_PORT, 0, buf, 4);
    sleep_us(10); gpio_put(imus[i].cs, 1);
    for (int j = 0; j < 4; j++) if (buf[j] != 0x00 && buf[j] != 0xFF) return true;
    return false;
}

// Switch & init a given IMU
bool select_imu(int i) {
    if (!available[i]) return false;
    if (cur >= 0) pico_bno08x_destroy(&active);
    const imu_cfg_t* c = &imus[i];
    if (!pico_bno08x_init(&active, c->rst, i)) return false;
    if (!pico_bno08x_begin_spi(&active, SPI_PORT, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN, c->cs, c->inten, 1000000)) return false;
    // enable both reports
    pico_bno08x_enable_report(&active, SH2_GYRO_INTEGRATED_RV, 50000);
    pico_bno08x_enable_report(&active, SH2_ROTATION_VECTOR,    100000);
    cur = i;
    printf("[SWITCH] %s active\n", c->name);
    return true;
}

int main() {
    init_hw();
    printf("=== Multi-IMU SPI Test ===\n");

    // detect available
    int count = 0;
    for (int i = 0; i < 3; i++) {
        available[i] = check_resp(i);
        printf("%s: %s\n", imus[i].name, available[i] ? "OK" : "NOPE");
        if (available[i]) count++;
    }
    if (!count) return -1;

    uint64_t t0 = to_ms_since_boot(get_absolute_time());
    int next = 0;

    while (1) {
        uint64_t t = to_ms_since_boot(get_absolute_time());
        if (t - t0 >= SWITCH_INTERVAL_MS) {
            // find next
            for (int k = 0; k < 3; k++) {
                int idx = (next + k) % 3;
                if (available[idx] && select_imu(idx)) {
                    next = (idx + 1) % 3;
                    t0 = t;
                    break;
                }
            }
        }
        if (cur >= 0) {
            // poll once
            for (int p = 0; p < POLL_ITERATIONS; p++) {
                pico_bno08x_service(&active);
                sh2_SensorValue_t v;
                if (pico_bno08x_get_sensor_event(&active, &v)) {
                    if (v.sensorId == SH2_ROTATION_VECTOR) {
                        printf("%s Q=[%.3f,%.3f,%.3f,%.3f]\n", imus[cur].name,
                               v.un.rotationVector.real,
                               v.un.rotationVector.i,
                               v.un.rotationVector.j,
                               v.un.rotationVector.k);
                    }
                    break;
                }
                sleep_ms(POLL_DELAY_MS);
            }
        }
    }
    return 0;
}
