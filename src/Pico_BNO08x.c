/*!
 *  @file Pico_BNO08x.c
 *
 *  Raspberry Pi Pico Driver for the BNO08x 9-DOF Orientation IMU Fusion
 *
 *  Ported from Adafruit BNO08x library to work with Raspberry Pi Pico SDK
 */

#include "Pico_BNO08x.h"
#include <string.h>

// Global pointer to current BNO08x instance for HAL callbacks
static Pico_BNO08x_t *current_bno = NULL;
static sh2_SensorValue_t *sensor_value = NULL;
static bool reset_occurred = false;

// Forward declarations for HAL functions
static bool pico_bno08x_init_common(Pico_BNO08x_t *bno);
static int i2c_hal_open(sh2_Hal_t *self);
static void i2c_hal_close(sh2_Hal_t *self);
static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

static int spi_hal_open(sh2_Hal_t *self);
static void spi_hal_close(sh2_Hal_t *self);
static int spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno);

static int uart_hal_open(sh2_Hal_t *self);
static void uart_hal_close(sh2_Hal_t *self);
static int uart_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int uart_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hardware_reset(Pico_BNO08x_t *bno);

/**
 * @brief Initialize BNO08x structure
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, int8_t reset_pin) {
    if (!bno) return false;
    
    // Clear the structure
    memset(bno, 0, sizeof(Pico_BNO08x_t));
    
    // Set default values
    bno->reset_pin = reset_pin;
    bno->int_pin = -1;
    bno->cs_pin = -1;
    bno->i2c_addr = BNO08x_I2CADDR_DEFAULT;
    bno->i2c_speed = 400000;  // 400kHz
    bno->spi_speed = 1000000; // 1MHz
    bno->uart_baudrate = 3000000; // 3Mbaud
    bno->interface_type = INTERFACE_NONE;
    bno->reset_occurred = false;
    
    // Setup HAL common functions
    bno->hal.getTimeUs = hal_get_time_us;
    
    return true;
}

/**
 * @brief Initialize I2C interface
 */
bool pico_bno08x_begin_i2c(Pico_BNO08x_t *bno, i2c_inst_t *i2c_port,
                           uint8_t sda_pin, uint8_t scl_pin,
                           uint8_t i2c_addr, uint32_t i2c_speed) {
    if (!bno || !i2c_port) return false;
    
    bno->i2c_port = i2c_port;
    bno->sda_pin = sda_pin;
    bno->scl_pin = scl_pin;
    bno->i2c_addr = i2c_addr;
    bno->i2c_speed = i2c_speed;
    bno->interface_type = INTERFACE_I2C;
    
    // Setup I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // Initialize I2C
    i2c_init(i2c_port, i2c_speed);
    
    // Setup HAL functions for I2C
    bno->hal.open = i2c_hal_open;
    bno->hal.close = i2c_hal_close;
    bno->hal.read = i2c_hal_read;
    bno->hal.write = i2c_hal_write;
    
    return pico_bno08x_init_common(bno);
}

/**
 * @brief Initialize SPI interface
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                          uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                          uint8_t cs_pin, uint8_t int_pin, uint32_t spi_speed) {
    if (!bno || !spi_port) return false;
    
    bno->spi_port = spi_port;
    bno->miso_pin = miso_pin;
    bno->mosi_pin = mosi_pin;
    bno->sck_pin = sck_pin;
    bno->cs_pin = cs_pin;
    bno->int_pin = int_pin;
    bno->spi_speed = spi_speed;
    bno->interface_type = INTERFACE_SPI;
    
    // Setup SPI pins
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    
    // Setup CS pin
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)
    
    // Setup interrupt pin
    gpio_init(int_pin);
    gpio_set_dir(int_pin, GPIO_IN);
    gpio_pull_up(int_pin);
    
    // Initialize SPI (Mode 3: CPOL=1, CPHA=1)
    spi_init(spi_port, spi_speed);
    spi_set_format(spi_port, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    // Setup HAL functions for SPI
    bno->hal.open = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read = spi_hal_read;
    bno->hal.write = spi_hal_write;
    
    return pico_bno08x_init_common(bno);
}

/**
 * @brief Initialize UART interface
 */
bool pico_bno08x_begin_uart(Pico_BNO08x_t *bno, uart_inst_t *uart_port,
                           uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate) {
    if (!bno || !uart_port) return false;
    
    bno->uart_port = uart_port;
    bno->tx_pin = tx_pin;
    bno->rx_pin = rx_pin;
    bno->uart_baudrate = baudrate;
    bno->interface_type = INTERFACE_UART;
    
    // Setup UART pins
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    // Initialize UART
    uart_init(uart_port, baudrate);
    uart_set_format(uart_port, 8, 1, UART_PARITY_NONE);
    
    // Setup HAL functions for UART
    bno->hal.open = uart_hal_open;
    bno->hal.close = uart_hal_close;
    bno->hal.read = uart_hal_read;
    bno->hal.write = uart_hal_write;
    
    return pico_bno08x_init_common(bno);
}

/**
 * @brief Common initialization after interface setup
 */
static bool pico_bno08x_init_common(Pico_BNO08x_t *bno) {
    current_bno = bno;
    
    // Perform hardware reset
    hardware_reset(bno);
    
    // Open SH2 interface
    int status = sh2_open(&bno->hal, hal_callback, NULL);
    if (status != SH2_OK) {
        printf("Failed to open SH2 interface: %d\n", status);
        return false;
    }
    
    // Get product IDs
    memset(&bno->prodIds, 0, sizeof(bno->prodIds));
    status = sh2_getProdIds(&bno->prodIds);
    if (status != SH2_OK) {
        printf("Failed to get product IDs: %d\n", status);
        return false;
    }
    
    // Register sensor callback
    sh2_setSensorCallback(sensor_handler, NULL);
    
    printf("BNO08x initialized successfully\n");
    return true;
}

/**
 * @brief Hardware reset
 */
static void hardware_reset(Pico_BNO08x_t *bno) {
    if (bno->reset_pin >= 0) {
        gpio_init(bno->reset_pin);
        gpio_set_dir(bno->reset_pin, GPIO_OUT);
        gpio_put(bno->reset_pin, 1);
        sleep_ms(10);
        gpio_put(bno->reset_pin, 0);
        sleep_ms(10);
        gpio_put(bno->reset_pin, 1);
        sleep_ms(10);
    }
}

void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno) {
    hardware_reset(bno);
}

/**
 * @brief Check if reset occurred
 */
bool pico_bno08x_was_reset(Pico_BNO08x_t *bno) {
    bool reset = bno->reset_occurred;
    bno->reset_occurred = false;
    return reset;
}

/**
 * @brief Enable sensor report
 */
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t config;
    
    // Configure sensor
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    config.reportInterval_us = interval_us;
    
    int status = sh2_setSensorConfig(sensor_id, &config);
    return (status == SH2_OK);
}

/**
 * @brief Get sensor event
 */
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value) {
    sensor_value = value;
    value->timestamp = 0;
    
    sh2_service();
    
    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        return false;
    }
    
    return true;
}

/**
 * @brief Service the sensor hub
 */
void pico_bno08x_service(Pico_BNO08x_t *bno) {
    sh2_service();
}

// HAL Implementation for I2C
static int i2c_hal_open(sh2_Hal_t *self) {
    if (!current_bno) return -1;
    
    // Send soft reset packet
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    int result = i2c_write_blocking(current_bno->i2c_port, current_bno->i2c_addr, 
                                   softreset_pkt, 5, false);
    if (result < 0) return -1;
    
    sleep_ms(300);
    return 0;
}

static void i2c_hal_close(sh2_Hal_t *self) {
    // Nothing to do for I2C close
}

static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    if (!current_bno || !pBuffer) return 0;
    
    uint8_t header[4];
    int result = i2c_read_blocking(current_bno->i2c_port, current_bno->i2c_addr, 
                                  header, 4, false);
    if (result < 0) return 0;
    
    uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_size &= ~0x8000; // Clear continuation bit
    
    if (packet_size > len) return 0;
    
    // Read the rest of the packet
    uint16_t cargo_remaining = packet_size;
    uint16_t read_pos = 0;
    
    // Copy header first
    if (cargo_remaining >= 4) {
        memcpy(pBuffer, header, 4);
        read_pos = 4;
        cargo_remaining -= 4;
    }
    
    // Read remaining data
    while (cargo_remaining > 0) {
        uint16_t to_read = (cargo_remaining > 252) ? 252 : cargo_remaining;
        result = i2c_read_blocking(current_bno->i2c_port, current_bno->i2c_addr,
                                  pBuffer + read_pos, to_read, false);
        if (result < 0) return 0;
        
        read_pos += to_read;
        cargo_remaining -= to_read;
    }
    
    return packet_size;
}

static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!current_bno || !pBuffer) return 0;
    
    int result = i2c_write_blocking(current_bno->i2c_port, current_bno->i2c_addr,
                                   pBuffer, len, false);
    return (result < 0) ? 0 : len;
}

// HAL Implementation for SPI
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    for (int i = 0; i < 500; i++) {
        if (!gpio_get(bno->int_pin)) return true;
        sleep_ms(1);
    }
    hardware_reset(bno);
    return false;
}

static int spi_hal_open(sh2_Hal_t *self) {
    if (!current_bno) return -1;
    
    return spi_hal_wait_for_int(current_bno) ? 0 : -1;
}

static void spi_hal_close(sh2_Hal_t *self) {
    // Nothing special to do for SPI close
}

static int spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    if (!current_bno || !pBuffer) return 0;
    
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    
    uint8_t dummy = 0x00;
    
    // Read header first
    gpio_put(current_bno->cs_pin, 0);
    spi_read_blocking(current_bno->spi_port, dummy, pBuffer, 4);
    gpio_put(current_bno->cs_pin, 1);
    
    uint16_t packet_size = (uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8);
    packet_size &= ~0x8000; // Clear continuation bit
    
    if (packet_size > len) return 0;
    
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    
    // Read full packet
    gpio_put(current_bno->cs_pin, 0);
    spi_read_blocking(current_bno->spi_port, dummy, pBuffer, packet_size);
    gpio_put(current_bno->cs_pin, 1);
    
    return packet_size;
}

static int spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!current_bno || !pBuffer) return 0;
    
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    
    gpio_put(current_bno->cs_pin, 0);
    spi_write_blocking(current_bno->spi_port, pBuffer, len);
    gpio_put(current_bno->cs_pin, 1);
    
    return len;
}

// HAL Implementation for UART
static int uart_hal_open(sh2_Hal_t *self) {
    if (!current_bno) return -1;
    
    // Flush input
    while (uart_is_readable(current_bno->uart_port)) {
        uart_getc(current_bno->uart_port);
    }
    
    // Send soft reset packet
    uint8_t softreset_pkt[] = {0x7E, 1, 5, 0, 1, 0, 1, 0x7E};
    for (int i = 0; i < sizeof(softreset_pkt); i++) {
        uart_putc(current_bno->uart_port, softreset_pkt[i]);
        sleep_ms(1);
    }
    
    return 0;
}

static void uart_hal_close(sh2_Hal_t *self) {
    // Nothing special to do for UART close
}

static int uart_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    if (!current_bno || !pBuffer) return 0;
    
    uint8_t c;
    uint16_t packet_size = 0;
    
    // Read packet start
    while (true) {
        if (!uart_is_readable(current_bno->uart_port)) continue;
        c = uart_getc(current_bno->uart_port);
        if (c == 0x7E) break;
    }
    
    // Read protocol ID
    while (!uart_is_readable(current_bno->uart_port)) tight_loop_contents();
    c = uart_getc(current_bno->uart_port);
    if (c == 0x7E) {
        while (!uart_is_readable(current_bno->uart_port)) tight_loop_contents();
        c = uart_getc(current_bno->uart_port);
        if (c != 0x01) return 0;
    } else if (c != 0x01) {
        return 0;
    }
    
    // Read packet data
    while (true) {
        while (!uart_is_readable(current_bno->uart_port)) tight_loop_contents();
        c = uart_getc(current_bno->uart_port);
        
        if (c == 0x7E) break;
        
        if (c == 0x7D) {
            // Escape sequence
            while (!uart_is_readable(current_bno->uart_port)) tight_loop_contents();
            c = uart_getc(current_bno->uart_port);
            c ^= 0x20;
        }
        
        if (packet_size < len) {
            pBuffer[packet_size] = c;
            packet_size++;
        }
    }
    
    return packet_size;
}

static int uart_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!current_bno || !pBuffer) return 0;
    
    // Start byte
    uart_putc(current_bno->uart_port, 0x7E);
    sleep_ms(1);
    
    // Protocol ID
    uart_putc(current_bno->uart_port, 0x01);
    sleep_ms(1);
    
    // Data with escaping
    for (int i = 0; i < len; i++) {
        uint8_t c = pBuffer[i];
        if ((c == 0x7E) || (c == 0x7D)) {
            uart_putc(current_bno->uart_port, 0x7D);
            sleep_ms(1);
            c ^= 0x20;
        }
        uart_putc(current_bno->uart_port, c);
        sleep_ms(1);
    }
    
    // End byte
    uart_putc(current_bno->uart_port, 0x7E);
    
    return len;
}

// Common HAL functions
static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    return time_us_32();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        reset_occurred = true;
        if (current_bno) {
            current_bno->reset_occurred = true;
        }
    }
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    if (!sensor_value) return;
    
    int rc = sh2_decodeSensorEvent(sensor_value, event);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n", rc);
        sensor_value->timestamp = 0;
    }
}