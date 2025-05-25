/**
 * BNO085 SPI Interface for Raspberry Pi Pico W
 * Implements communication with BNO085 over SPI to read quaternion data
 */

#include "bno085_spi.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Global state
static spi_inst_t* spi_instance;
static uint cs_gpio, intr_gpio, wake_gpio;
static uint8_t receive_buffer[128];
static uint8_t transmit_buffer[128];
static uint8_t sh2_command_seq = 0;
static uint8_t sh2_report_seq = 0;
static uint8_t channel_command = 2;  // Channel for sensor hub control
static uint8_t channel_reports = 3;  // Channel for input sensor reports

// Function prototypes for internal use
static bool bno085_spi_read_packet(uint8_t* buffer, size_t* length);
static bool bno085_spi_write_packet(uint8_t channel, const uint8_t* data, size_t length);
static bool bno085_spi_wait_for_interrupt(uint32_t timeout_ms);
static bool bno085_spi_wake_device(void);
static bool bno085_spi_parse_quaternion(const uint8_t* buffer, size_t length, bno085_spi_quaternion_t* quat);
static void increment_seq(uint8_t* seq);

// Initialize the BNO085 sensor with SPI
bool bno085_spi_init(spi_inst_t* spi_inst, uint miso_pin, uint mosi_pin, 
                     uint sck_pin, uint cs_pin, uint intr_pin, uint wake_pin) {
    
    // Save parameters
    spi_instance = spi_inst;
    cs_gpio = cs_pin;
    intr_gpio = intr_pin;
    wake_gpio = wake_pin;
    
    printf("BNO085 SPI: Initializing SPI interface...\n");
    
    // Initialize SPI at 3MHz (max for BNO085)
    spi_init(spi_instance, 3000000);
    
    // Set SPI mode 3 (CPOL=1, CPHA=1) - clock idles high, data sampled on rising edge
    spi_set_format(spi_instance, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    // Set up SPI GPIO pins
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    
    // Set up CS pin as GPIO output (active low)
    gpio_init(cs_gpio);
    gpio_set_dir(cs_gpio, GPIO_OUT);
    gpio_put(cs_gpio, 1); // CS idle high
    
    // Set up interrupt pin as input with pull-up
    gpio_init(intr_gpio);
    gpio_set_dir(intr_gpio, GPIO_IN);
    gpio_pull_up(intr_gpio);
    
    // Set up wake pin as GPIO output (active low)
    gpio_init(wake_gpio);
    gpio_set_dir(wake_gpio, GPIO_OUT);
    gpio_put(wake_gpio, 1); // Wake idle high
    
    printf("BNO085 SPI: Waiting for device to boot...\n");
    sleep_ms(100); // Wait for device boot
    
    // Wait for initial interrupt indicating device is ready
    if (!bno085_spi_wait_for_interrupt(3000)) {
        printf("BNO085 SPI: Timeout waiting for device ready signal\n");
        return false;
    }
    
    // Wake the device and read initial advertisement
    if (!bno085_spi_wake_device()) {
        printf("BNO085 SPI: Failed to wake device\n");
        return false;
    }
    
    size_t length = sizeof(receive_buffer);
    if (bno085_spi_read_packet(receive_buffer, &length)) {
        printf("BNO085 SPI: Received initial packet (%d bytes)\n", (int)length);
    }
    
    // Send product ID request to verify communication
    printf("BNO085 SPI: Requesting product ID...\n");
    uint8_t product_id_request[] = {0xF9, 0x00};
    if (!bno085_spi_write_packet(channel_command, product_id_request, sizeof(product_id_request))) {
        printf("BNO085 SPI: Failed to send product ID request\n");
        return false;
    }
    
    // Wait for response
    if (!bno085_spi_wait_for_interrupt(1000)) {
        printf("BNO085 SPI: Timeout waiting for product ID response\n");
        return false;
    }
    
    if (!bno085_spi_wake_device()) {
        printf("BNO085 SPI: Failed to wake device for reading response\n");
        return false;
    }
    
    length = sizeof(receive_buffer);
    if (!bno085_spi_read_packet(receive_buffer, &length)) {
        printf("BNO085 SPI: Failed to read product ID response\n");
        return false;
    }
    
    // Parse product ID response
    bool valid_id = false;
    for (size_t i = 0; i < length - 15; i++) {
        if (receive_buffer[i] == 0xF8) {  // Product ID response
            valid_id = true;
            uint8_t sw_major = receive_buffer[i+2];
            uint8_t sw_minor = receive_buffer[i+3];
            uint16_t sw_patch = receive_buffer[i+12] | (receive_buffer[i+13] << 8);
            printf("BNO085 SPI: Product ID verified, SW Version: %d.%d.%d\n", 
                   sw_major, sw_minor, sw_patch);
            break;
        }
    }
    
    if (!valid_id) {
        printf("BNO085 SPI: Invalid or no product ID response\n");
        return false;
    }
    
    printf("BNO085 SPI: Initialization successful\n");
    return true;
}

// Enable rotation vector reports
bool bno085_spi_enable_rotation_vector(uint32_t interval_us) {
    printf("BNO085 SPI: Enabling rotation vector reports at %ld us interval\n", (long)interval_us);
    
    // Prepare Set Feature command for Rotation Vector (0x05)
    uint8_t set_feature_cmd[17] = {
        0xFD,                   // Set Feature command
        0x05,                   // Rotation Vector report ID
        0x00,                   // Feature flags (none)
        0x00, 0x00,             // Change sensitivity (not used)
        (uint8_t)(interval_us & 0xFF),
        (uint8_t)((interval_us >> 8) & 0xFF),
        (uint8_t)((interval_us >> 16) & 0xFF),
        (uint8_t)((interval_us >> 24) & 0xFF),
        0x00, 0x00, 0x00, 0x00, // Batch interval (not used)
        0x00, 0x00, 0x00, 0x00  // Sensor-specific config (not used)
    };
    
    // Send the Set Feature command
    if (!bno085_spi_write_packet(channel_command, set_feature_cmd, sizeof(set_feature_cmd))) {
        printf("BNO085 SPI: Failed to send Set Feature command\n");
        return false;
    }
    
    // Wait for response
    if (!bno085_spi_wait_for_interrupt(500)) {
        printf("BNO085 SPI: Timeout waiting for Set Feature response\n");
        return false;
    }
    
    if (!bno085_spi_wake_device()) {
        printf("BNO085 SPI: Failed to wake device for Set Feature response\n");
        return false;
    }
    
    // Read response
    size_t length = sizeof(receive_buffer);
    if (!bno085_spi_read_packet(receive_buffer, &length)) {
        printf("BNO085 SPI: Failed to read Set Feature response\n");
        return false;
    }
    
    // Parse response - look for Get Feature Response (0xFC)
    bool valid_response = false;
    for (size_t i = 0; i < length - 8; i++) {
        if (receive_buffer[i] == 0xFC && receive_buffer[i+1] == 0x05) {
            valid_response = true;
            uint32_t actual_interval = receive_buffer[i+5] | 
                                     (receive_buffer[i+6] << 8) | 
                                     (receive_buffer[i+7] << 16) | 
                                     (receive_buffer[i+8] << 24);
            printf("BNO085 SPI: Rotation Vector enabled at %ld us interval\n", (long)actual_interval);
            break;
        }
    }
    
    if (!valid_response) {
        printf("BNO085 SPI: Invalid Set Feature response\n");
        return false;
    }
    
    return true;
}

// Read quaternion data
bool bno085_spi_read_quaternion(bno085_spi_quaternion_t* quat) {
    // Check for new data
    if (!bno085_spi_wait_for_interrupt(50)) {
        return false; // No new data
    }
    
    if (!bno085_spi_wake_device()) {
        return false;
    }
    
    size_t length = sizeof(receive_buffer);
    if (!bno085_spi_read_packet(receive_buffer, &length)) {
        return false;
    }
    
    return bno085_spi_parse_quaternion(receive_buffer, length, quat);
}

// Wake the BNO085 device by pulsing the wake pin
static bool bno085_spi_wake_device(void) {
    // Assert wake signal (active low)
    gpio_put(wake_gpio, 0);
    sleep_us(10); // Hold for at least 10us
    gpio_put(wake_gpio, 1);
    
    // Wait for interrupt to indicate device is awake
    return bno085_spi_wait_for_interrupt(200); // 200ms timeout
}

// Wait for interrupt signal
static bool bno085_spi_wait_for_interrupt(uint32_t timeout_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
        // Check if interrupt pin is low (active low)
        if (!gpio_get(intr_gpio)) {
            return true;
        }
        sleep_ms(1);
    }
    
    return false; // Timeout
}

// Read a packet via SPI
static bool bno085_spi_read_packet(uint8_t* buffer, size_t* length) {
    uint8_t header[4];
    
    // Assert CS
    gpio_put(cs_gpio, 0);
    sleep_us(1);
    
    // Read SHTP header (4 bytes)
    spi_read_blocking(spi_instance, 0x00, header, 4);
    
    // Calculate packet length from header
    size_t packet_length = header[0] | ((header[1] & 0x7F) << 8);
    
    if (packet_length < 4) {
        gpio_put(cs_gpio, 1); // Deassert CS
        printf("BNO085 SPI: Invalid packet length: %d\n", (int)packet_length);
        return false;
    }
    
    packet_length -= 4; // Subtract header length
    
    if (packet_length > *length) {
        gpio_put(cs_gpio, 1); // Deassert CS
        printf("BNO085 SPI: Packet too large (%d bytes)\n", (int)packet_length);
        return false;
    }
    
    if (packet_length > 0) {
        // Read the packet payload
        spi_read_blocking(spi_instance, 0x00, buffer, packet_length);
    }
    
    // Deassert CS
    gpio_put(cs_gpio, 1);
    
    *length = packet_length;
    return true;
}

// Write a packet via SPI
static bool bno085_spi_write_packet(uint8_t channel, const uint8_t* data, size_t length) {
    if (length > sizeof(transmit_buffer) - 4) {
        return false;
    }
    
    // Prepare SHTP header
    size_t total_length = length + 4;
    transmit_buffer[0] = (uint8_t)(total_length & 0xFF);
    transmit_buffer[1] = (uint8_t)((total_length >> 8) & 0x7F);
    transmit_buffer[2] = channel;
    
    // Set sequence number
    if (channel == channel_command) {
        transmit_buffer[3] = sh2_command_seq;
        increment_seq(&sh2_command_seq);
    } else {
        transmit_buffer[3] = sh2_report_seq;
        increment_seq(&sh2_report_seq);
    }
    
    // Copy payload
    memcpy(transmit_buffer + 4, data, length);
    
    // Assert CS
    gpio_put(cs_gpio, 0);
    sleep_us(1);
    
    // Write the packet
    spi_write_blocking(spi_instance, transmit_buffer, total_length);
    
    // Deassert CS
    gpio_put(cs_gpio, 1);
    
    return true;
}

// Parse quaternion data from packet
static bool bno085_spi_parse_quaternion(const uint8_t* buffer, size_t length, bno085_spi_quaternion_t* quat) {
    // Look for rotation vector report (0x05)
    for (size_t i = 0; i < length; i++) {
        if (buffer[i] == 0x05 && i + 13 < length) {
            size_t data_start = i + 4;
            
            if (data_start + 9 >= length) continue;
            
            // Extract quaternion components (Q14 fixed point)
            int16_t i_real = (int16_t)(buffer[data_start] | (buffer[data_start+1] << 8));
            int16_t i_i = (int16_t)(buffer[data_start+2] | (buffer[data_start+3] << 8));
            int16_t i_j = (int16_t)(buffer[data_start+4] | (buffer[data_start+5] << 8));
            int16_t i_k = (int16_t)(buffer[data_start+6] | (buffer[data_start+7] << 8));
            
            // Convert from Q14 fixed-point to float
            quat->real = (float)i_real / 16384.0f;
            quat->i = (float)i_i / 16384.0f;
            quat->j = (float)i_j / 16384.0f;
            quat->k = (float)i_k / 16384.0f;
            
            // Extract accuracy if available
            if (data_start + 9 < length) {
                uint16_t accuracy_raw = (buffer[data_start+8] | (buffer[data_start+9] << 8));
                quat->accuracy_rad = (float)accuracy_raw / 4096.0f;
            } else {
                quat->accuracy_rad = 0.0f;
            }
            
            quat->accuracy_status = buffer[i+2] & 0x03;
            
            return true;
        }
    }
    
    return false;
}

// Increment sequence number
static void increment_seq(uint8_t* seq) {
    (*seq)++;
}