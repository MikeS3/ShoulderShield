/**
 * Simple BNO085 Example for Original Raspberry Pi Pico
 * 
 * A minimal example showing how to read quaternion data from the BNO085 sensor.
 * This can be used as a starting point for understanding the BNO085 interface.
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "bno085.h"

// Pin definitions (adjust these based on your wiring)
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define BNO085_INT_PIN 6

// Helper function to convert quaternion to Euler angles (roll, pitch, yaw)
void quaternion_to_euler(const bno085_quaternion_t* q, float* roll, float* pitch, float* yaw) {
    // Roll (rotation around X-axis)
    float sinr_cosp = 2 * (q->real * q->i + q->j * q->k);
    float cosr_cosp = 1 - 2 * (q->i * q->i + q->j * q->j);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (rotation around Y-axis)
    float sinp = 2 * (q->real * q->j - q->k * q->i);
    if (fabsf(sinp) >= 1) {
        *pitch = copysignf(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (rotation around Z-axis)
    float siny_cosp = 2 * (q->real * q->k + q->i * q->j);
    float cosy_cosp = 1 - 2 * (q->j * q->j + q->k * q->k);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

// Convert radians to degrees
float rad_to_deg(float rad) {
    return rad * 180.0f / M_PI;
}

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB connection
    
    printf("\n=== BNO085 Quaternion Reader Example (Original Pico) ===\n");
    
    // Initialize LED for status indication (original Pico uses GPIO 25)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    // Initialize BNO085
    printf("Initializing BNO085...\n");
    if (!bno085_init(i2c0, I2C_SDA_PIN, I2C_SCL_PIN, BNO085_INT_PIN, BNO085_I2C_ADDR_SA0_0)) {
        printf("ERROR: Failed to initialize BNO085!\n");
        printf("Check your wiring and I2C address.\n");
        
        // Blink LED rapidly to indicate error
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }
    
    // Enable rotation vector reports at 50Hz (20,000 microseconds)
    printf("Enabling rotation vector reports...\n");
    if (!bno085_enable_rotation_vector(20000)) {
        printf("ERROR: Failed to enable rotation vector reports!\n");
        
        // Blink LED slowly to indicate configuration error
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(500);
        }
    }
    
    printf("BNO085 is ready! Reading quaternion data...\n");
    printf("Format: Quat(w,x,y,z) | Euler(roll,pitch,yaw) | Accuracy\n");
    printf("-----------------------------------------------------\n");
    
    bno085_quaternion_t quat;
    uint32_t sample_count = 0;
    uint32_t last_led_toggle = 0;
    bool led_state = false;
    
    while (true) {
        if (bno085_read_quaternion(&quat)) {
            sample_count++;
            
            // Convert quaternion to Euler angles for easier interpretation
            float roll, pitch, yaw;
            quaternion_to_euler(&quat, &roll, &pitch, &yaw);
            
            // Print data every 25 samples (roughly once per second at 50Hz)
            if (sample_count % 25 == 0) {
                printf("Q(%6.3f,%6.3f,%6.3f,%6.3f) | E(%6.1f°,%6.1f°,%6.1f°) | Acc:%d\n",
                       quat.real, quat.i, quat.j, quat.k,
                       rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw),
                       quat.accuracy_status);
                
                // Show accuracy status
                switch (quat.accuracy_status) {
                    case BNO085_ACCURACY_UNRELIABLE:
                        printf("  Status: UNRELIABLE - Move device to calibrate\n");
                        break;
                    case BNO085_ACCURACY_LOW:
                        printf("  Status: LOW accuracy\n");
                        break;
                    case BNO085_ACCURACY_MEDIUM:
                        printf("  Status: MEDIUM accuracy\n");
                        break;
                    case BNO085_ACCURACY_HIGH:
                        printf("  Status: HIGH accuracy\n");
                        break;
                }
            }
            
            // Toggle LED every second to show we're receiving data
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_led_toggle > 1000) {
                led_state = !led_state;
                gpio_put(PICO_DEFAULT_LED_PIN, led_state);
                last_led_toggle = now;
            }
        }
        
        // Small delay to prevent overwhelming the system
        sleep_ms(5);
    }
    
    return 0;
}