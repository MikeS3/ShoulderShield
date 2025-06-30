#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "Pico_BNO08x.h"

// Pin definitions
#define SPI0_PORT       spi0
#define SPI0_MISO_PIN   16
#define SPI0_MOSI_PIN   19
#define SPI0_SCK_PIN    18
#define CS1_PIN         17
#define INT1_PIN        20
#define RESET1_PIN      15

#define SPI1_PORT       spi1
#define SPI1_MISO_PIN   12
#define SPI1_MOSI_PIN   11
#define SPI1_SCK_PIN    10
#define CS2_PIN         13
#define INT2_PIN        9
#define RESET2_PIN      14

#define I2C_PORT        i2c1
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_ADDR        BNO08x_I2CADDR_DEFAULT
#define RESET3_PIN      7

// Global IMU instances
Pico_BNO08x_t imu1, imu2, imu3;
bool imu1_ok = false, imu2_ok = false, imu3_ok = false;

// Initialize IMUs
bool init_imu_spi(Pico_BNO08x_t *imu, spi_inst_t *spi_port, 
                  uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                  int reset_pin, int cs_pin, int int_pin, int instance_id) {
    
    if (!pico_bno08x_init(imu, reset_pin, instance_id)) return false;
    return pico_bno08x_begin_spi(imu, spi_port, miso_pin, mosi_pin, sck_pin, 
                                cs_pin, int_pin, 1000000);
}

bool init_imu_i2c(Pico_BNO08x_t *imu, i2c_inst_t *i2c_port,
                  uint8_t sda_pin, uint8_t scl_pin, uint8_t i2c_addr,
                  int reset_pin, int instance_id) {
    
    if (!pico_bno08x_init(imu, reset_pin, instance_id)) return false;
    return pico_bno08x_begin_i2c(imu, i2c_port, sda_pin, scl_pin, 
                                i2c_addr, 400000);
}

void enable_sensors(Pico_BNO08x_t *imu) {
    pico_bno08x_enable_report(imu, SH2_ROTATION_VECTOR, 100000);  // 10 Hz
}

void read_and_print_imu(Pico_BNO08x_t *imu, const char *name) {
    sh2_SensorValue_t sensor_value;
    if (pico_bno08x_get_sensor_event(imu, &sensor_value)) {
        if (sensor_value.sensorId == SH2_ROTATION_VECTOR) {
            printf("%s Quaternion: i=%.3f j=%.3f k=%.3f real=%.3f\n",
                   name,
                   sensor_value.un.rotationVector.i,
                   sensor_value.un.rotationVector.j,
                   sensor_value.un.rotationVector.k,
                   sensor_value.un.rotationVector.real);
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("Starting Multi-Instance 3-IMU Test\n");
    printf("IMU1: SPI0, IMU2: SPI1, IMU3: I2C1 (addr=0x%02X)\n\n", I2C_ADDR);
    
    // Initialize IMU1 on SPI0
    printf("Initializing IMU1 (SPI0)...\n");
    imu1_ok = init_imu_spi(&imu1, SPI0_PORT, SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_SCK_PIN,
                           RESET1_PIN, CS1_PIN, INT1_PIN, 1);
    if (imu1_ok) {
        enable_sensors(&imu1);
        sleep_ms(500);
    }
    
    // Initialize IMU2 on SPI1
    printf("Initializing IMU2 (SPI1)...\n");
    imu2_ok = init_imu_spi(&imu2, SPI1_PORT, SPI1_MISO_PIN, SPI1_MOSI_PIN, SPI1_SCK_PIN,
                           RESET2_PIN, CS2_PIN, INT2_PIN, 2);
    if (imu2_ok) {
        enable_sensors(&imu2);
        sleep_ms(500);
    }
    
    // Initialize IMU3 on I2C
    printf("Initializing IMU3 (I2C, addr=0x%02X)...\n", I2C_ADDR);
    imu3_ok = init_imu_i2c(&imu3, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_ADDR,
                           RESET3_PIN, 3);
    if (imu3_ok) {
        enable_sensors(&imu3);
        sleep_ms(500);
    }
    
    printf("\nStarting sensor reading loop...\n\n");
    
    // Main loop with round-robin reading
    while (true) {
        if (imu1_ok) read_and_print_imu(&imu1, "IMU1(SPI0)");
        if (imu2_ok) read_and_print_imu(&imu2, "IMU2(SPI1)");
        if (imu3_ok) read_and_print_imu(&imu3, "IMU3(I2C)");
        
        sleep_ms(500); // Half second between readings
    }
    
    return 0;
}