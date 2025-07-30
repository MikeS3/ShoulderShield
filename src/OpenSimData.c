#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

// SPI Configuration
#define SPI_PORT        spi0
#define SPI_MISO_PIN    16
#define SPI_MOSI_PIN    19
#define SPI_SCK_PIN     18

// IMU0 Pins
#define CS1_PIN         17
#define INT1_PIN        20
#define RESET1_PIN      15

// IMU1 Pins
#define CS2_PIN         13
#define INT2_PIN        12
#define RESET2_PIN      14

// IMU2 Pins
#define CS3_PIN         9
#define INT3_PIN        10
#define RESET3_PIN      11

#define SWITCH_INTERVAL_MS 50
#define POLL_ITERATIONS     1
#define POLL_DELAY_MS       10
#define DATA_RATE (1000.0f/POLL_DELAY_MS)

typedef struct {
    uint cs;
    uint inten;
    uint rst;
    const char* name;
    const char* opensim_name;
} imu_cfg_t;

static const imu_cfg_t imus[3] = {
    {CS1_PIN, INT1_PIN, RESET1_PIN, "IMU0", "scapula_imu"},
    {CS2_PIN, INT2_PIN, RESET2_PIN, "IMU1", "sternum_imu"},
    {CS3_PIN, INT3_PIN, RESET3_PIN, "IMU2", "humerus_imu"}
};

Pico_BNO08x_t active;
Pico_BNO08x_t IMUs[3];
int cur = -1;
bool available[3] = { false };
uint64_t t0 = 0;

void init_hw() {
    stdio_init_all();
    sleep_ms(8000);  // Wait for USB serial

    spi_init(SPI_PORT, 3000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);

    for (int i = 0; i < 3; i++) {
        gpio_init(imus[i].cs);    gpio_set_dir(imus[i].cs, GPIO_OUT);  gpio_put(imus[i].cs, 1);
        gpio_init(imus[i].inten); gpio_set_dir(imus[i].inten, GPIO_IN); gpio_pull_down(imus[i].inten);
        gpio_init(imus[i].rst);   gpio_set_dir(imus[i].rst, GPIO_OUT); gpio_put(imus[i].rst, 0);
    }
    sleep_ms(100);
    for (int i = 0; i < 3; i++) {
        gpio_put(imus[i].rst, 1);  // Reset IMUs
        sleep_ms(50);
    }
    sleep_ms(200);
}

bool check_resp(int i) {
    uint8_t buf[4] = {0};
    gpio_put(imus[i].cs, 0); sleep_us(5);
    spi_read_blocking(SPI_PORT, 0, buf, 4);
    sleep_us(5); gpio_put(imus[i].cs, 1);
    for (int j = 0; j < 4; j++) if (buf[j] != 0x00 && buf[j] != 0xFF) return true;
    return false;
}

bool select_imu(int i) {
    if (!available[i]) return false;
    if (cur >= 0) pico_bno08x_destroy(&active);
    const imu_cfg_t* c = &imus[i];
    if (!pico_bno08x_init(&active, c->rst, i)) return false;
    if (!pico_bno08x_begin_spi(&active, SPI_PORT,
        SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
        c->cs, c->inten, 3000000)) return false;

    pico_bno08x_enable_report(&active, SH2_ROTATION_VECTOR, 50000);
    pico_bno08x_enable_report(&active, SH2_ACCELEROMETER, 50000);
    pico_bno08x_enable_report(&active, SH2_GYROSCOPE_CALIBRATED, 50000);

    cur = i;
    return true;
}
void init_imus(Pico_BNO08x_t* IMUs_h, int num_IMUs)
{
    imu_cfg_t* cfg = &imus[0];
    for(int i=0; i < num_IMUs; i++){ //for each IMU
    cfg = &imus[i];
    if(pico_bno08x_init(&IMUs_h[i], cfg->rst, i))printf("Initializing IMU %i\n", i);

    if(pico_bno08x_begin_spi(&IMUs_h[i], SPI_PORT,
        SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN,
        cfg->cs, cfg->inten, 3000000)) printf("Initializing IMU %i SPI\n", i);

    if(pico_bno08x_enable_report(&IMUs_h[i], SH2_ROTATION_VECTOR, 5000)) printf("Enabled Quaternion reports for IMU %i", i);

     sh2_closeInstance(IMUs_h[i].sh2_instance); //close the instance after its generated
    }
   
}
void print_data_collection_header() {
    printf("<data_start>rate:%.2f\t\n", DATA_RATE);
}
//sh2_openInstance
typedef struct {
    float time;
    bool has_quat, has_accel, has_gyro;
    float q[4], acc[3], gyr[3];
} imu_data_t;

int main() {
    init_hw();

    // Check which IMUs are connected
    int count = 0;
    for (int i = 0; i < 3; i++) {
        available[i] = check_resp(i);
        printf("# %s (%s): %s\n", imus[i].name, imus[i].opensim_name, available[i] ? "OK" : "NOPE");
        if (available[i]) count++;
    }
    if (!count) {
        printf("# ERROR: No IMUs detected\n");
        return -1;
    }

    init_imus(IMUs, sizeof(IMUs)/sizeof(Pico_BNO08x_t)); // init imu instances.

    print_data_collection_header();
    uint64_t last_switch = to_ms_since_boot(get_absolute_time());
    int next = 0;
    uint64_t t0 = last_switch;
    imu_data_t current_data = {0};

    while (1) {
        uint64_t now = to_ms_since_boot(get_absolute_time());
        printf("now - last switch %.7f\n", now - last_switch);
        if (now - last_switch >= SWITCH_INTERVAL_MS) {
            for (int k = 0; k < 3; k++) {
                int idx = (next + k) % 3;
                if (available[idx]) {
                    cur = idx;
                    next = (idx + 1) % 3;
                    last_switch = now;
                    current_data = (imu_data_t){0};
                    current_data.time = (now - t0) / 1000.0f;
                    break;
                }
            }
        }


        if (cur >= 0) {
            //printf("current IMU is %i\n", cur);
            //sh2_openInstance(IMUs[cur].sh2_instance, &IMUs[cur].hal, sensor_handler, NULL ); //reopen the instance before servicing
            //uint64_t before = to_ms_since_boot(get_absolute_time());
            sh2_openInstance(IMUs[cur].sh2_instance, &IMUs[cur].hal, hal_callback, &IMUs[cur]);
            sh2_setSensorCallbackInstance(IMUs[cur].sh2_instance, sensor_handler, &IMUs[cur]);
            //printf("open instance and set callback delay: %.7f", to_ms_since_boot(get_absolute_time())- before);
            for (int p = 0; p < POLL_ITERATIONS; p++) {
                pico_bno08x_service(&IMUs[cur]);
                sh2_SensorValue_t v;

                if (pico_bno08x_get_sensor_event(&IMUs[cur], &v)) {
                    //printf("IMU %i has an event\n", cur);
                    current_data.time = (to_ms_since_boot(get_absolute_time()) - t0) / 1000.0f;

                    switch (v.sensorId) {
                        case SH2_ROTATION_VECTOR:
                            current_data.q[0] = v.un.rotationVector.real;
                            current_data.q[1] = v.un.rotationVector.i;
                            current_data.q[2] = v.un.rotationVector.j;
                            current_data.q[3] = v.un.rotationVector.k;
                            current_data.has_quat = true;
                            break;

                        case SH2_ACCELEROMETER:
                            current_data.acc[0] = v.un.accelerometer.x;
                            current_data.acc[1] = v.un.accelerometer.y;
                            current_data.acc[2] = v.un.accelerometer.z;
                            current_data.has_accel = true;
                            break;

                        case SH2_GYROSCOPE_CALIBRATED:
                            current_data.gyr[0] = v.un.gyroscope.x;
                            current_data.gyr[1] = v.un.gyroscope.y;
                            current_data.gyr[2] = v.un.gyroscope.z;
                            current_data.has_gyro = true;
                            break;
                    }

                    if (current_data.has_quat) {
                        printf("%.3f,%s,%.6f,%.6f,%.6f,%.6f\n",
                            current_data.time,
                            imus[cur].opensim_name,
                            current_data.q[0], current_data.q[1], current_data.q[2], current_data.q[3]);
                        break;
                    }
                }
                sleep_ms(POLL_DELAY_MS);
            }
            sh2_closeInstance(IMUs[cur].sh2_instance); //close the instance after reading
        }
    }

    return 0;
}
