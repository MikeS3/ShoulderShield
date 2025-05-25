/**
 * BNO085 SPI Example for Raspberry Pi Pico W
 * 
 * Demonstrates reading quaternion data from BNO085 using SPI interface.
 * This is the most complex but highest-performance interface option.
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "bno085_spi.h"

// SPI pin definitions (adjust based on your wiring)
#define SPI_MISO_PIN 16
#define SPI_MOSI_PIN 19
#define SPI_SCK_PIN  18
#define SPI_CS_PIN   17
#define BNO085_INT_PIN 6
#define BNO085_WAKE_PIN 7  // Connected to BNO085 PS0/WAKE pin

// Helper function to convert quaternion to Euler angles
void quaternion_to_euler(const bno085_spi_quaternion_t* q, float* roll, float* pitch, float* yaw) {
    // Roll (rotation around X-axis)
    float sinr_cosp = 2 * (q->real * q->i + q->j * q->k);
    float cosr_cosp = 1 - 2 * (q->i * q->i + q->j * q->j);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (rotation around Y-axis)
    float sinp = 2 * (q->real * q->j - q->k * q->i);
    if (fabsf(sinp) >= 1) {
        *pitch = copysignf(M_PI / 2, sinp);
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (rotation around Z-axis)
    float siny_cosp = 2 * (q->real * q->k + q->i * q->j);
    float cosy_cosp = 1 - 2 * (q->j * q->j + q->k * q->k);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

float rad_to_deg(float rad) {
    return rad * 180.0f / M_PI;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== BNO085 SPI Quaternion Reader Example ===\n");
    printf("This example uses SPI communication for highest performance.\n\n");
    
    // Initialize LED (original Pico uses GPIO 25)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    // Display wiring instructions
    printf("Required BNO085 connections for SPI mode:\n");
    printf("  VDD     -> 3.3V\n");
    printf("  GND     -> GND\n");
    printf("  PS1     -> 3.3V (SPI mode select)\n");
    printf("  PS0     -> GPIO %d (WAKE signal)\n", BNO085_WAKE_PIN);
    printf("  H_MISO  -> GPIO %d (SPI MISO)\n", SPI_MISO_PIN);
    printf("  H_MOSI  -> GPIO %d (SPI MOSI)\n", SPI_MOSI_PIN);
    printf("  H_SCK   -> GPIO %d (SPI SCK)\n", SPI_SCK_PIN);
    printf("  H_CSN   -> GPIO %d (SPI CS)\n", SPI_CS_PIN);
    printf("  H_INTN  -> GPIO %d (Interrupt)\n", BNO085_INT_PIN);
    printf("  BOOTN   -> 3.3V (Normal boot)\n\n");
    
    // Initialize BNO085 with SPI
    printf("Initializing BNO085 with SPI...\n");
    if (!bno085_spi_init(spi0, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN, 
                         SPI_CS_PIN, BNO085_INT_PIN, BNO085_WAKE_PIN)) {
        printf("ERROR: Failed to initialize BNO085 via SPI!\n");
        printf("Check your wiring and pin connections.\n");
        
        // Rapid blink for SPI init error
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }
    
    // Enable rotation vector at high rate (10ms = 100Hz)
    printf("Enabling rotation vector reports at 100Hz...\n");
    if (!bno085_spi_enable_rotation_vector(10000)) {
        printf("ERROR: Failed to enable rotation vector reports!\n");
        
        // Slow blink for config error
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(500);
        }
    }
    
    printf("BNO085 SPI ready! Reading quaternion data at high speed...\n");
    printf("Format: Quaternion(w,x,y,z) | Euler(roll,pitch,yaw) | Accuracy\n");
    printf("---------------------------------------------------------------\n");
    
    bno085_spi_quaternion_t quat;
    uint32_t sample_count = 0;
    uint32_t last_led_toggle = 0;
    uint32_t last_print_time = 0;
    bool led_state = false;
    
    // Performance tracking
    uint32_t last_rate_calc = 0;
    uint32_t samples_since_last_calc = 0;
    
    while (true) {
        if (bno085_spi_read_quaternion(&quat)) {
            sample_count++;
            samples_since_last_calc++;
            
            uint32_t now = to_ms_since_boot(get_absolute_time());
            
            // Print data every 500ms
            if (now - last_print_time > 500) {
                float roll, pitch, yaw;
                quaternion_to_euler(&quat, &roll, &pitch, &yaw);
                
                printf("[%6lu] Q(%6.3f,%6.3f,%6.3f,%6.3f) | E(%6.1f°,%6.1f°,%6.1f°) | Acc:%d",
                       sample_count,
                       quat.real, quat.i, quat.j, quat.k,
                       rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw),
                       quat.accuracy_status);
                
                // Show accuracy status
                switch (quat.accuracy_status) {
                    case BNO085_ACCURACY_UNRELIABLE:
                        printf(" (UNRELIABLE)\n");
                        break;
                    case BNO085_ACCURACY_LOW:
                        printf(" (LOW)\n");
                        break;
                    case BNO085_ACCURACY_MEDIUM:
                        printf(" (MEDIUM)\n");
                        break;
                    case BNO085_ACCURACY_HIGH:
                        printf(" (HIGH)\n");
                        break;
                }
                
                last_print_time = now;
            }
            
            // Calculate data rate every 5 seconds
            if (now - last_rate_calc > 5000) {
                float rate = (float)samples_since_last_calc / 5.0f;
                printf(">>> SPI Data rate: %.1f Hz (target: 100 Hz)\n", rate);
                
                last_rate_calc = now;
                samples_since_last_calc = 0;
            }
            
            // Toggle LED every second
            if (now - last_led_toggle > 1000) {
                led_state = !led_state;
                gpio_put(PICO_DEFAULT_LED_PIN, led_state);
                last_led_toggle = now;
            }
        }
        
        sleep_us(100); // Small delay to prevent CPU overload
    }
    
    return 0;
}