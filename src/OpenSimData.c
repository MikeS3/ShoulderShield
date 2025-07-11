// === OpenSim-Compatible Multi-IMU SPI Data Collection ===

#include <stdio.h>
#include "pico/stdlib.h"
#include "Pico_BNO08x.h"

// SPI Configuration pins
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

// Timing parameters
#define SWITCH_INTERVAL_MS 1000    // switch every 1 second
#define POLL_ITERATIONS     8      // polls per switch
#define POLL_DELAY_MS       40     // ms between polls

// IMU configuration struct
typedef struct {
    uint cs;
    uint inten;
    uint rst;
    const char* name;
    const char* opensim_name;  // OpenSim-friendly sensor names
} imu_cfg_t;

static const imu_cfg_t imus[3] = {
    {CS1_PIN, INT1_PIN, RESET1_PIN, "IMU0", "ShoulderBlade_imu"},
    {CS2_PIN, INT2_PIN, RESET2_PIN, "IMU1", "Sterum_imu"},
    {CS3_PIN, INT3_PIN, RESET3_PIN, "IMU2", "Humerus_imu"}
};

Pico_BNO08x_t active;
int cur = -1;
bool available[3] = { false };
uint64_t t0 = 0;

void init_hw() {
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial

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
    
    // Enable multiple sensor reports for OpenSim
    pico_bno08x_enable_report(&active, SH2_ROTATION_VECTOR, 50000);      // Quaternion
    pico_bno08x_enable_report(&active, SH2_ACCELEROMETER, 50000);        // Linear acceleration
    pico_bno08x_enable_report(&active, SH2_GYROSCOPE_CALIBRATED, 50000); // Angular velocity
    
    cur = i;
    return true;
}

void print_opensim_header() {
    printf("# OpenSim IMU Data Collection\n");
    printf("# Generated from Pico Multi-IMU System\n");
    printf("# Units: time(s), quaternions(unitless), accel(m/s²), gyro(rad/s)\n");
    printf("# Format compatible with OpenSim IMU Inverse Kinematics\n");
    printf("time,sensor_name,q0,q1,q2,q3,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z\n");
}

int main() {
    init_hw();
    print_opensim_header();

    // detect available IMUs
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

    uint64_t last_switch = to_ms_since_boot(get_absolute_time());
    int next = 0;
    t0 = last_switch;  // mark start time

    // Data storage for synchronized output
    typedef struct {
        float time;
        bool has_quat, has_accel, has_gyro;
        float q[4], acc[3], gyr[3];
    } imu_data_t;
    
    imu_data_t current_data = {0};

    while (1) {
        uint64_t now = to_ms_since_boot(get_absolute_time());
        
        // Switch IMU periodically
        if (now - last_switch >= SWITCH_INTERVAL_MS) {
            for (int k = 0; k < 3; k++) {
                int idx = (next + k) % 3;
                if (available[idx] && select_imu(idx)) {
                    next = (idx + 1) % 3;
                    last_switch = now;
                    // Reset data collection for new sensor
                    current_data = (imu_data_t){0};
                    current_data.time = (now - t0) / 1000.0f;
                    break;
                }
            }
        }

        if (cur >= 0) {
            bool data_ready = false;
            
            for (int p = 0; p < POLL_ITERATIONS; p++) {
                pico_bno08x_service(&active);
                sh2_SensorValue_t v;
                
                if (pico_bno08x_get_sensor_event(&active, &v)) {
                    current_data.time = (to_ms_since_boot(get_absolute_time()) - t0) / 1000.0f;
                    
                    switch (v.sensorId) {
                        case SH2_ROTATION_VECTOR:
                            current_data.q[0] = v.un.rotationVector.real;  // w
                            current_data.q[1] = v.un.rotationVector.i;     // x
                            current_data.q[2] = v.un.rotationVector.j;     // y
                            current_data.q[3] = v.un.rotationVector.k;     // z
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
                    
                    // Output data when we have at least quaternion (minimum for OpenSim)
                    if (current_data.has_quat) {
                        printf("%.3f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                            current_data.time,
                            imus[cur].opensim_name,
                            current_data.q[0], current_data.q[1], current_data.q[2], current_data.q[3],
                            current_data.has_accel ? current_data.acc[0] : 0.0f,
                            current_data.has_accel ? current_data.acc[1] : 0.0f,
                            current_data.has_accel ? current_data.acc[2] : 0.0f,
                            current_data.has_gyro ? current_data.gyr[0] : 0.0f,
                            current_data.has_gyro ? current_data.gyr[1] : 0.0f,
                            current_data.has_gyro ? current_data.gyr[2] : 0.0f);
                        
                        data_ready = true;
                        break;
                    }
                }
                sleep_ms(POLL_DELAY_MS);
            }
        }
    }
    return 0;
}