/**
 * BNO085 Sensor Interface for Raspberry Pi Pico W
 * Implements communication with BNO085 over I2C to read quaternion data
 */

#include "bno085.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Global state
static i2c_inst_t* i2c_instance;
static uint8_t bno085_address;
static uint8_t receive_buffer[128];
static uint8_t transmit_buffer[128];
static uint8_t sh2_command_seq = 0;
static uint8_t sh2_report_seq = 0;
static uint8_t report_seq_number = 0;
static uint8_t channel_command = 2;  // Channel for sensor hub control
static uint8_t channel_reports = 3;  // Channel for input sensor reports
static uint intr_gpio;               // Interrupt pin

// Function prototypes for internal use
static bool bno085_read_packet(uint8_t* buffer, size_t* length);
static bool bno085_write_packet(uint8_t channel, const uint8_t* data, size_t length);
static bool bno085_wait_for_interrupt(uint32_t timeout_ms);
static bool bno085_parse_quaternion(const uint8_t* buffer, size_t length, bno085_quaternion_t* quat);
static void incrementSh2Seq(uint8_t* seq);

// Initialize the BNO085 sensor
bool bno085_init(i2c_inst_t* i2c_inst, uint i2c_sda_pin, uint i2c_scl_pin, uint intr_pin, uint8_t address) {
    // Save parameters
    i2c_instance = i2c_inst;
    bno085_address = address;
    intr_gpio = intr_pin;
    
    // Initialize I2C pins
    i2c_init(i2c_instance, 400 * 1000);  // 400 kHz
    gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_pin);
    gpio_pull_up(i2c_scl_pin);
    
    // Initialize interrupt pin
    gpio_init(intr_pin);
    gpio_set_dir(intr_pin, GPIO_IN);
    gpio_pull_up(intr_pin);
    
    printf("BNO085: Waiting for device to boot...\n");
    sleep_ms(100);  // Wait for device to boot
    
    // Wait for initial interrupt to indicate device is ready
    if (!bno085_wait_for_interrupt(2000)) {
        printf("BNO085: Timeout waiting for device ready signal\n");
        return false;
    }
    
    // Read initial advertisement packet
    size_t length = sizeof(receive_buffer);
    if (bno085_read_packet(receive_buffer, &length)) {
        printf("BNO085: Received initial packet (%d bytes)\n", (int)length);
    }
    
    // Send product ID request to verify communication
    printf("BNO085: Requesting product ID...\n");
    uint8_t product_id_request[] = {0xF9, 0x00};
    if (!bno085_write_packet(channel_command, product_id_request, sizeof(product_id_request))) {
        printf("BNO085: Failed to send product ID request\n");
        return false;
    }
    
    // Wait for response
    if (!bno085_wait_for_interrupt(1000)) {
        printf("BNO085: Timeout waiting for product ID response\n");
        return false;
    }
    
    length = sizeof(receive_buffer);
    if (!bno085_read_packet(receive_buffer, &length)) {
        printf("BNO085: Failed to read product ID response\n");
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
            printf("BNO085: Product ID verified, SW Version: %d.%d.%d\n", 
                   sw_major, sw_minor, sw_patch);
            break;
        }
    }
    
    if (!valid_id) {
        printf("BNO085: Invalid or no product ID response\n");
        return false;
    }
    
    printf("BNO085: Initialization successful\n");
    return true;
}

// Configure the BNO085 to report rotation vector (quaternion) data
bool bno085_enable_rotation_vector(uint32_t interval_us) {
    printf("BNO085: Enabling rotation vector reports at %ld us interval\n", (long)interval_us);
    
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
    if (!bno085_write_packet(channel_command, set_feature_cmd, sizeof(set_feature_cmd))) {
        printf("BNO085: Failed to send Set Feature command\n");
        return false;
    }
    
    // Wait for response
    if (!bno085_wait_for_interrupt(500)) {
        printf("BNO085: Timeout waiting for Set Feature response\n");
        return false;
    }
    
    // Read response
    size_t length = sizeof(receive_buffer);
    if (!bno085_read_packet(receive_buffer, &length)) {
        printf("BNO085: Failed to read Set Feature response\n");
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
            printf("BNO085: Rotation Vector enabled at %ld us interval\n", (long)actual_interval);
            break;
        }
    }
    
    if (!valid_response) {
        printf("BNO085: Invalid Set Feature response\n");
        return false;
    }
    
    return true;
}

// Read quaternion data from the BNO085
bool bno085_read_quaternion(bno085_quaternion_t* quat) {
    // Check for new data
    if (!bno085_wait_for_interrupt(50)) {
        // No new data available within timeout
        return false;
    }
    
    size_t length = sizeof(receive_buffer);
    if (!bno085_read_packet(receive_buffer, &length)) {
        return false;
    }
    
    return bno085_parse_quaternion(receive_buffer, length, quat);
}

// Internal function to parse quaternion data from a response packet
static bool bno085_parse_quaternion(const uint8_t* buffer, size_t length, bno085_quaternion_t* quat) {
    // Look for rotation vector report (0x05) in the packet
    // The packet may contain multiple reports, so we need to scan through it
    for (size_t i = 0; i < length; i++) {
        if (buffer[i] == 0x05 && i + 13 < length) {  // Rotation vector report + minimum data size
            // Found rotation vector data
            // Skip sequence number (i+1) and status/delay (i+2, i+3)
            size_t data_start = i + 4;
            
            if (data_start + 9 >= length) continue;  // Not enough data
            
            // Extract quaternion components (Q14 fixed point format)
            int16_t i_real = (int16_t)(buffer[data_start] | (buffer[data_start+1] << 8));
            int16_t i_i = (int16_t)(buffer[data_start+2] | (buffer[data_start+3] << 8));
            int16_t i_j = (int16_t)(buffer[data_start+4] | (buffer[data_start+5] << 8));
            int16_t i_k = (int16_t)(buffer[data_start+6] | (buffer[data_start+7] << 8));
            
            // Convert from Q14 fixed-point to float
            quat->real = (float)i_real / 16384.0f;
            quat->i = (float)i_i / 16384.0f;
            quat->j = (float)i_j / 16384.0f;
            quat->k = (float)i_k / 16384.0f;
            
            // Extract accuracy estimate if available
            if (data_start + 9 < length) {
                uint16_t accuracy_raw = (buffer[data_start+8] | (buffer[data_start+9] << 8));
                quat->accuracy_rad = (float)accuracy_raw / 4096.0f;  // Q12 format
            } else {
                quat->accuracy_rad = 0.0f;
            }
            
            // Extract status (accuracy bits)
            quat->accuracy_status = buffer[i+2] & 0x03;
            
            return true;
        }
    }
    
    return false;  // Rotation vector data not found
}

// Wait for the interrupt pin to signal new data
static bool bno085_wait_for_interrupt(uint32_t timeout_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
        // Check if interrupt pin is low (active low)
        if (!gpio_get(intr_gpio)) {
            return true;
        }
        sleep_ms(1);  // Small delay to prevent busy waiting
    }
    
    return false;  // Timeout
}

// Read a packet from the BNO085 sensor
static bool bno085_read_packet(uint8_t* buffer, size_t* length) {
    uint8_t header[4];
    
    // Read the SHTP header (4 bytes)
    int result = i2c_read_blocking(i2c_instance, bno085_address, header, 4, false);
    if (result != 4) {
        printf("BNO085: Failed to read SHTP header (got %d bytes)\n", result);
        return false;
    }
    
    // Calculate packet length from header (little-endian, first 2 bytes)
    size_t packet_length = header[0] | ((header[1] & 0x7F) << 8);  // Mask continuation bit
    
    if (packet_length < 4) {
        printf("BNO085: Invalid packet length: %d\n", (int)packet_length);
        return false;
    }
    
    // Adjust length to exclude header
    packet_length -= 4;
    
    if (packet_length > *length) {
        printf("BNO085: Packet too large (%d bytes, buffer is %d)\n", (int)packet_length, (int)*length);
        return false;
    }
    
    if (packet_length == 0) {
        *length = 0;
        return true;  // Empty packet is valid
    }
    
    // Read the packet payload
    result = i2c_read_blocking(i2c_instance, bno085_address, buffer, packet_length, false);
    if (result != (int)packet_length) {
        printf("BNO085: Failed to read packet payload (expected %d, got %d)\n", (int)packet_length, result);
        return false;
    }
    
    *length = packet_length;
    return true;
}

// Write a packet to the BNO085 sensor
static bool bno085_write_packet(uint8_t channel, const uint8_t* data, size_t length) {
    if (length > sizeof(transmit_buffer) - 4) {
        printf("BNO085: Packet too large for transmit buffer\n");
        return false;
    }
    
    // Prepare SHTP header
    size_t total_length = length + 4;  // Data + 4-byte header
    transmit_buffer[0] = (uint8_t)(total_length & 0xFF);
    transmit_buffer[1] = (uint8_t)((total_length >> 8) & 0x7F);  // No continuation bit
    transmit_buffer[2] = channel;
    
    // Set sequence number based on channel
    if (channel == channel_command) {
        transmit_buffer[3] = sh2_command_seq;
        incrementSh2Seq(&sh2_command_seq);
    } else {
        transmit_buffer[3] = sh2_report_seq;
        incrementSh2Seq(&sh2_report_seq);
    }
    
    // Copy payload
    memcpy(transmit_buffer + 4, data, length);
    
    // Write the packet
    int result = i2c_write_blocking(i2c_instance, bno085_address, transmit_buffer, total_length, false);
    if (result != (int)total_length) {
        printf("BNO085: Failed to write packet (expected %d, wrote %d)\n", (int)total_length, result);
        return false;
    }
    
    return true;
}

// Increment sequence number (wraps at 256)
static void incrementSh2Seq(uint8_t* seq) {
    (*seq)++;
}