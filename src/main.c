/**
 * ShoulderShield - Shoulder Cuff Injury Prevention Device
 * 
 * Main application for reading quaternion data from BNO085 sensor
 * to detect shoulder movement and prevent injuries.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno085.h"

// Pin definitions
#define I2C_SDA_PIN 4  // GPIO pin for I2C SDA
#define I2C_SCL_PIN 5  // GPIO pin for I2C SCL
#define BNO085_INT_PIN 6  // GPIO pin for BNO085 interrupt

// LED control helper functions
static bool led_state = false;

void toggle_led() {
    led_state = !led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
}

void init_led() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

int main() {
    // Initialize standard I/O over USB
    stdio_init_all();
    
    // Wait for USB connection to establish
    sleep_ms(2000);
    
    printf("\nShoulderShield - Shoulder Cuff Injury Prevention Device\n");
    printf("Initializing...\n");
    
    // Initialize LED
    init_led();
    
    // Blink LED to indicate startup
    for (int i = 0; i < 3; i++) {
        toggle_led();
        sleep_ms(200);
        toggle_led();
        sleep_ms(200);
    }
    
    // Initialize BNO085 sensor
    printf("Initializing BNO085 sensor...\n");
    if (!bno085_init(i2c0, I2C_SDA_PIN, I2C_SCL_PIN, BNO085_INT_PIN, BNO085_I2C_ADDR_SA0_0)) {
        printf("Failed to initialize BNO085 sensor!\n");
        // Error indicator: rapid blinking
        while (true) {
            toggle_led();
            sleep_ms(100);
        }
    }
    
    printf("BNO085 initialized successfully\n");
    
    // Configure BNO085 to report rotation vector data at 100Hz (10,000 microseconds)
    printf("Enabling rotation vector reports...\n");
    if (!bno085_enable_rotation_vector(10000)) {
        printf("Failed to enable rotation vector reports!\n");
        // Error indicator: slow blinking
        while (true) {
            toggle_led();
            sleep_ms(500);
        }
    }
    
    printf("Rotation vector reports enabled\n");
    printf("Starting main loop\n");
    
    // Main loop - read quaternion data and process it
    bno085_quaternion_t quaternion;
    uint32_t last_print_time = 0;
    
    while (true) {
        // Read quaternion data from BNO085
        if (bno085_read_quaternion(&quaternion)) {
            // Blink LED to indicate successful reading
            toggle_led();
            
            // Print quaternion data at a reasonable rate (once per second)
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - last_print_time > 1000) {
                printf("Quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f, accuracy=%u\n",
                       quaternion.real, quaternion.i, quaternion.j, quaternion.k,
                       quaternion.accuracy_status);
                last_print_time = current_time;
            }
            
            // TODO: Add your shoulder movement detection algorithm here
            // This would analyze the quaternion data to detect potentially harmful movements
        }
        
        // Small delay to prevent overwhelming the CPU
        sleep_ms(10);
    }
    
    return 0;
}