#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "Pico_BNO08x.h"

// SPI0 Pins (IMU1)
#define SPI0_PORT       spi0
#define SPI0_MISO_PIN   16
#define SPI0_MOSI_PIN   19
#define SPI0_SCK_PIN    18

// SPI1 Pins (IMU2)
#define SPI1_PORT       spi1
#define SPI1_MISO_PIN   12
#define SPI1_MOSI_PIN   11
#define SPI1_SCK_PIN    10

// I2C Pins (IMU3)
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_ADDR        BNO08x_I2CADDR_DEFAULT

// IMU1 Pins (SPI0)
#define CS1_PIN         17
#define INT1_PIN        20
#define RESET1_PIN      15

// IMU2 Pins (SPI1)
#define CS2_PIN         13
#define INT2_PIN        9
#define RESET2_PIN      14

// IMU3 Pins (I2C)
#define RESET3_PIN      7
// Note: SA0 pin tied to GND in hardware for I2C address 0x4A

// Global IMU instances and status flags
Pico_BNO08x_t imu1, imu2, imu3;
bool imu1_ok = false, imu2_ok = false, imu3_ok = false;

// Set up all required GPIOs
void configure_gpio() {
    // SPI0 Chip Select: default HIGH (inactive)
    gpio_init(CS1_PIN); 
    gpio_set_dir(CS1_PIN, GPIO_OUT); 
    gpio_put(CS1_PIN, 1);

    // SPI1 Chip Select: default HIGH (inactive)
    gpio_init(CS2_PIN); 
    gpio_set_dir(CS2_PIN, GPIO_OUT); 
    gpio_put(CS2_PIN, 1);

    // Reset Pins: default OUTPUT LOW
    gpio_init(RESET1_PIN); gpio_set_dir(RESET1_PIN, GPIO_OUT);
    gpio_init(RESET2_PIN); gpio_set_dir(RESET2_PIN, GPIO_OUT);
    gpio_init(RESET3_PIN); gpio_set_dir(RESET3_PIN, GPIO_OUT);

    // INT Pins: INPUT w/ pull-up (only for SPI sensors)
    gpio_init(INT1_PIN); gpio_set_dir(INT1_PIN, GPIO_IN); gpio_pull_up(INT1_PIN);
    gpio_init(INT2_PIN); gpio_set_dir(INT2_PIN, GPIO_IN); gpio_pull_up(INT2_PIN);
}

void init_spi0_bus() {
    // Initialize SPI0 bus
    spi_init(SPI0_PORT, 1000000);
    spi_set_format(SPI0_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    // Set up SPI0 GPIO functions
    gpio_set_function(SPI0_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_SCK_PIN, GPIO_FUNC_SPI);
}

void init_spi1_bus() {
    // Initialize SPI1 bus
    spi_init(SPI1_PORT, 1000000);
    spi_set_format(SPI1_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    // Set up SPI1 GPIO functions
    gpio_set_function(SPI1_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
}

void init_i2c_bus() {
    // Initialize I2C1 bus
    i2c_init(I2C_PORT, 400000);  // 400kHz
    
    // Set up I2C GPIO functions
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

// Initialize IMU on SPI
bool init_imu_spi(Pico_BNO08x_t *imu, spi_inst_t *spi_port, 
                  uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                  int reset_pin, int cs_pin, int int_pin, int instance_id) {
    // Reset this IMU
    gpio_put(reset_pin, 0);
    sleep_ms(10);
    gpio_put(reset_pin, 1);
    sleep_ms(50); // Wait for IMU boot

    if (!pico_bno08x_init(imu, reset_pin, instance_id)) return false;
    return pico_bno08x_begin_spi(imu, spi_port, miso_pin, mosi_pin, sck_pin, 
                                cs_pin, int_pin, 1000000); // 1 MHz SPI
}

// Initialize IMU on I2C1
bool init_imu_i2c(Pico_BNO08x_t *imu, i2c_inst_t *i2c_port,
                  uint8_t sda_pin, uint8_t scl_pin, uint8_t i2c_addr,
                  int reset_pin, int instance_id) {
    // Reset this IMU
    gpio_put(reset_pin, 0);
    sleep_ms(10);
    gpio_put(reset_pin, 1);
    sleep_ms(50); // Wait for IMU boot

    if (!pico_bno08x_init(imu, reset_pin, instance_id)) return false;
    return pico_bno08x_begin_i2c(imu, i2c_port, sda_pin, scl_pin, 
                                i2c_addr, 400000); // 400kHz I2C
}

// Enable sensor reports: rotation, accel, gyro
void enable_sensors(Pico_BNO08x_t *imu) {
    pico_bno08x_enable_report(imu, SH2_ROTATION_VECTOR, 10000);        // 100 Hz
    pico_bno08x_enable_report(imu, SH2_ACCELEROMETER, 10000);          // 100 Hz
    pico_bno08x_enable_report(imu, SH2_GYROSCOPE_CALIBRATED, 10000);   // 100 Hz
}

void read_and_print_imu(Pico_BNO08x_t *imu, const char *name) {
    pico_bno08x_service(imu);
    
    sh2_SensorValue_t sensor_value;
    if (pico_bno08x_get_sensor_event(imu, &sensor_value)) {
        switch (sensor_value.sensorId) {
            case SH2_ROTATION_VECTOR:
                printf("%s Rotation - i:%.3f j:%.3f k:%.3f real:%.3f accuracy:%d\n",
                       name,
                       sensor_value.un.rotationVector.i,
                       sensor_value.un.rotationVector.j,
                       sensor_value.un.rotationVector.k,
                       sensor_value.un.rotationVector.real,
                       sensor_value.un.rotationVector.accuracy);
                break;
            case SH2_ACCELEROMETER:
                printf("%s Accel - X:%.3f Y:%.3f Z:%.3f m/s²\n",
                       name,
                       sensor_value.un.accelerometer.x,
                       sensor_value.un.accelerometer.y,
                       sensor_value.un.accelerometer.z);
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                printf("%s Gyro - X:%.3f Y:%.3f Z:%.3f rad/s\n",
                       name,
                       sensor_value.un.gyroscope.x,
                       sensor_value.un.gyroscope.y,
                       sensor_value.un.gyroscope.z);
                break;
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Give time for USB serial to connect
    
    printf("Starting Mixed Interface 3-IMU Test\n");
    printf("IMU1: SPI0, IMU2: SPI1, IMU3: I2C1 (SA0=GND, addr=0x%02X)\n\n", I2C_ADDR);
    
    // Configure all GPIOs first
    configure_gpio();
    
    // Initialize communication buses
    init_spi0_bus();
    init_spi1_bus();
    init_i2c_bus();
    
    printf("Buses initialized...\n");
    
    // Initialize IMU1 on SPI0
    printf("Initializing IMU1 (SPI0)...\n");
    imu1_ok = init_imu_spi(&imu1, SPI0_PORT, SPI0_MISO_PIN, SPI0_MOSI_PIN, SPI0_SCK_PIN,
                           RESET1_PIN, CS1_PIN, INT1_PIN, 1);
    if (imu1_ok) {
        printf("IMU1 initialized successfully!\n");
        enable_sensors(&imu1);
    } else {
        printf("Failed to initialize IMU1\n");
    }
    
    // Initialize IMU2 on SPI1
    printf("Initializing IMU2 (SPI1)...\n");
    imu2_ok = init_imu_spi(&imu2, SPI1_PORT, SPI1_MISO_PIN, SPI1_MOSI_PIN, SPI1_SCK_PIN,
                           RESET2_PIN, CS2_PIN, INT2_PIN, 2);
    if (imu2_ok) {
        printf("IMU2 initialized successfully!\n");
        enable_sensors(&imu2);
    } else {
        printf("Failed to initialize IMU2\n");
    }
    
    // Initialize IMU3 on I2C (address BNO08x_I2CADDR_DEFAULT due to SA0 tied to GND)
    printf("Initializing IMU3 (I2C, addr=0x%02X)...\n", I2C_ADDR);
    imu3_ok = init_imu_i2c(&imu3, I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_ADDR,
                           RESET3_PIN, 3);
    if (imu3_ok) {
        printf("IMU3 initialized successfully!\n");
        enable_sensors(&imu3);
    } else {
        printf("Failed to initialize IMU3\n");
    }
    
    printf("\nStarting sensor reading loop...\n\n");
    
    // Main loop - read all active IMUs
    while (true) {
        if (imu1_ok) read_and_print_imu(&imu1, "IMU1(SPI0)");
        if (imu2_ok) read_and_print_imu(&imu2, "IMU2(SPI1)");
        if (imu3_ok) read_and_print_imu(&imu3, "IMU3(I2C)");
        
        sleep_ms(50); // Small delay between readings
    }
    
    return 0;
}