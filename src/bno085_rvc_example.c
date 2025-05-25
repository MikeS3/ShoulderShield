/**
 * BNO085 UART-RVC Mode Example for Raspberry Pi Pico
 * 
 * This example demonstrates how to read orientation and acceleration data
 * from the BNO085 sensor using the simple UART-RVC mode.
 * 
 * In RVC mode, the BNO085 automatically sends data packets at 100Hz
 * without needing complex initialization commands.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "bno085_rvc.h"

// Pin definitions for UART connection to BNO085
#define UART_TX_PIN 0  // Pico TX -> BNO085 RX (not used in RVC mode, but connect anyway)
#define UART_RX_PIN 1  // Pico RX -> BNO085 TX
#define UART_BAUD 115200  // Standard baud rate for RVC mode

// Helper function to print orientation in a readable format
void print_orientation(const bno085_rvc_data_t *data) {
    printf("Orientation: Yaw=%6.2f° Pitch=%6.2f° Roll=%6.2f°", 
           data->yaw, data->pitch, data->roll);
}

// Helper function to print acceleration in a readable format  
void print_acceleration(const bno085_rvc_data_t *data) {
    printf("Accel: X=%6.3f Y=%6.3f Z=%6.3f m/s²", 
           data->x_accel, data->y_accel, data->z_accel);
}

// Calculate magnitude of acceleration vector
float calculate_accel_magnitude(const bno085_rvc_data_t *data) {
    return sqrtf(data->x_accel * data->x_accel + 
                 data->y_accel * data->y_accel + 
                 data->z_accel * data->z_accel);
}

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB connection
    
    printf("\n=== BNO085 UART-RVC Mode Example ===\n");
    printf("This example reads orientation and acceleration data from BNO085\n");
    printf("using the simple UART-RVC (Robot Vacuum Cleaner) mode.\n\n");
    
    // Initialize LED for status indication (original Pico uses GPIO 25)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    // Initialize BNO085 in RVC mode
    printf("Initializing BNO085 in UART-RVC mode...\n");
    printf("Make sure BNO085 is wired for RVC mode:\n");
    printf("  PS1 = GND (pin 5)\n");
    printf("  PS0 = 3.3V (pin 6)\n");
    printf("  BNO085 TX -> Pico GPIO %d\n", UART_RX_PIN);
    printf("  BNO085 RX -> Pico GPIO %d (optional)\n\n", UART_TX_PIN);
    
    if (!bno085_rvc_init(uart0, UART_TX_PIN, UART_RX_PIN, UART_BAUD)) {
        printf("ERROR: Failed to initialize UART for BNO085 RVC mode!\n");
        return -1;
    }
    
    printf("Initialization complete. The BNO085 should start sending data automatically.\n");
    printf("Data format: [Orientation] | [Acceleration] | [Total G-force]\n");
    printf("Note: It may take a few seconds for the first data to appear.\n");
    printf("--------------------------------------------------------------\n");
    
    bno085_rvc_data_t sensor_data;
    uint32_t packet_count = 0;
    uint32_t last_led_toggle = 0;
    uint32_t last_print_time = 0;
    bool led_state = false;
    
    // Variables for data rate calculation
    uint32_t last_rate_calc = 0;
    uint32_t packets_since_last_calc = 0;
    
    while (true) {
        if (bno085_rvc_read(&sensor_data)) {
            packet_count++;
            packets_since_last_calc++;
            
            uint32_t now = to_ms_since_boot(get_absolute_time());
            
            // Print data every 500ms (about every 50 packets at 100Hz)
            if (now - last_print_time > 500) {
                printf("[%6lu] ", packet_count);
                print_orientation(&sensor_data);
                printf(" | ");
                print_acceleration(&sensor_data);
                printf(" | Total: %5.2f m/s²\n", calculate_accel_magnitude(&sensor_data));
                
                last_print_time = now;
            }
            
            // Calculate and display data rate every 5 seconds
            if (now - last_rate_calc > 5000) {
                float rate = (float)packets_since_last_calc / 5.0f;
                printf(">>> Data rate: %.1f Hz (expected: 100 Hz)\n", rate);
                
                last_rate_calc = now;
                packets_since_last_calc = 0;
            }
            
            // Toggle LED every second to show we're receiving data
            if (now - last_led_toggle > 1000) {
                led_state = !led_state;
                gpio_put(PICO_DEFAULT_LED_PIN, led_state);
                last_led_toggle = now;
            }
            
        } else {
            // No data received, check if we should indicate an error
            uint32_t now = to_ms_since_boot(get_absolute_time());
            
            // If we haven't received data for more than 5 seconds, blink rapidly
            if (packet_count == 0 && now > 10000) { // After 10 seconds of no data
                static uint32_t error_blink = 0;
                if (now - error_blink > 200) {
                    led_state = !led_state;
                    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
                    error_blink = now;
                    
                    // Print troubleshooting info occasionally
                    if ((now / 5000) % 2 == 0) { // Every 10 seconds
                        printf("No data received yet. Check:\n");
                        printf("  1. BNO085 power connections (VDD=3.3V, GND)\n");
                        printf("  2. PS1=GND, PS0=3.3V for RVC mode\n");
                        printf("  3. BNO085 TX connected to Pico GPIO %d\n", UART_RX_PIN);
                        printf("  4. UART baud rate matches (%d)\n", UART_BAUD);
                    }
                }
            }
        }
        
        sleep_ms(1); // Small delay to prevent overwhelming the CPU
    }
    
    return 0;
}