/*
 * Pico_BNO08x.c - Multi-IMU support for BNO08x on Raspberry Pi Pico
 * This version includes proper initialization and basic sensor simulation
 */

#include "Pico_BNO08x.h"
#include <string.h>
#include <math.h>

#define MAX_BNO08X_IMUS 3

// Queue management functions
static bool bno08x_event_queue_push(bno08x_event_queue_t *queue, sh2_SensorValue_t *val) {
    uint8_t next = (queue->head + 1) % BNO08X_EVENT_QUEUE_SIZE;
    if (next == queue->tail) return false; // Queue full
    queue->queue[queue->head] = *val;
    queue->head = next;
    return true;
}

static bool bno08x_event_queue_pop(bno08x_event_queue_t *queue, sh2_SensorValue_t *val) {
    if (queue->head == queue->tail) return false; // Queue empty
    *val = queue->queue[queue->tail];
    queue->tail = (queue->tail + 1) % BNO08X_EVENT_QUEUE_SIZE;
    return true;
}

// Simulate sensor data generation (replace with actual BNO08x communication later)
static void generate_test_sensor_data(Pico_BNO08x_t *bno, uint8_t imu_index) {
    static uint32_t last_gen_time[MAX_BNO08X_IMUS] = {0};
    uint32_t now = time_us_32() / 1000; // milliseconds
    
    // Generate data every 100ms (10Hz)
    if (now - last_gen_time[imu_index] < 100) return;
    last_gen_time[imu_index] = now;
    
    sh2_SensorValue_t sensor_value;
    memset(&sensor_value, 0, sizeof(sensor_value));
    
    // Generate fake quaternion data
    float time_sec = now / 1000.0f;
    float phase = imu_index * 2.0f * M_PI / 3.0f; // Different phase for each IMU
    
    sensor_value.sensorId = SH2_ROTATION_VECTOR;
    sensor_value.timestamp = now * 1000; // Convert to microseconds
    sensor_value.status = 3; // Good accuracy
    
    // Generate rotating quaternion
    sensor_value.un.rotationVector.real = cosf(time_sec * 0.1f + phase);
    sensor_value.un.rotationVector.i = sinf(time_sec * 0.1f + phase) * 0.5f;
    sensor_value.un.rotationVector.j = sinf(time_sec * 0.15f + phase) * 0.3f;
    sensor_value.un.rotationVector.k = sinf(time_sec * 0.12f + phase) * 0.2f;
    sensor_value.un.rotationVector.accuracy = 0.1f;
    
    bno08x_event_queue_push(&bno->event_queue, &sensor_value);
    
    // Generate fake accelerometer data
    sensor_value.sensorId = SH2_ACCELEROMETER;
    sensor_value.un.accelerometer.x = sinf(time_sec * 0.5f + phase) * 2.0f;
    sensor_value.un.accelerometer.y = cosf(time_sec * 0.3f + phase) * 1.5f;
    sensor_value.un.accelerometer.z = 9.81f + sinf(time_sec * 0.2f + phase) * 0.5f;
    
    bno08x_event_queue_push(&bno->event_queue, &sensor_value);
    
    // Generate fake gyroscope data
    sensor_value.sensorId = SH2_GYROSCOPE_CALIBRATED;
    sensor_value.un.gyroscope.x = sinf(time_sec * 0.8f + phase) * 0.1f;
    sensor_value.un.gyroscope.y = cosf(time_sec * 0.6f + phase) * 0.08f;
    sensor_value.un.gyroscope.z = sinf(time_sec * 0.4f + phase) * 0.05f;
    
    bno08x_event_queue_push(&bno->event_queue, &sensor_value);
    
    // Generate fake magnetometer data
    sensor_value.sensorId = SH2_MAGNETIC_FIELD_CALIBRATED;
    sensor_value.un.magneticField.x = 25.0f + sinf(time_sec * 0.1f + phase) * 5.0f;
    sensor_value.un.magneticField.y = 30.0f + cosf(time_sec * 0.15f + phase) * 3.0f;
    sensor_value.un.magneticField.z = 45.0f + sinf(time_sec * 0.08f + phase) * 2.0f;
    
    bno08x_event_queue_push(&bno->event_queue, &sensor_value);
}

// Basic SPI communication setup
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                           uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                           uint8_t cs_pin, uint8_t int_pin, uint8_t reset_pin,
                           uint32_t spi_speed) {
    
    printf("Initializing BNO08x on CS pin %d...\n", cs_pin);
    
    bno->spi_port = spi_port;
    bno->cs_pin = cs_pin;
    bno->int_pin = int_pin;
    bno->reset_pin = reset_pin;
    bno->spi_speed = spi_speed;
    bno->interface_type = INTERFACE_SPI;

    // Initialize GPIO pins
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)

    gpio_init(reset_pin);
    gpio_set_dir(reset_pin, GPIO_OUT);
    gpio_put(reset_pin, 0); // Reset active
    sleep_ms(10);
    gpio_put(reset_pin, 1); // Release reset
    sleep_ms(100);

    // Initialize interrupt pin as input
    gpio_init(int_pin);
    gpio_set_dir(int_pin, GPIO_IN);
    gpio_pull_up(int_pin);

    // Initialize SPI
    spi_init(spi_port, spi_speed);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);

    // Initialize event queue
    memset(&bno->event_queue, 0, sizeof(bno->event_queue));

    printf("BNO08x SPI initialization complete (CS: %d, INT: %d, RST: %d)\n", 
           cs_pin, int_pin, reset_pin);

    // TODO: Add actual BNO08x chip communication and sh2 HAL setup here
    // For now, we'll simulate sensor data
    
    return true;
}

// Service function - reads from hardware and populates queue
void pico_bno08x_service(Pico_BNO08x_t *bno) {
    static uint8_t imu_counter = 0;
    
    if (!bno || !bno->initialized) return;
    
    // TODO: Replace this with actual BNO08x communication
    // For now, generate test data
    generate_test_sensor_data(bno, imu_counter % MAX_BNO08X_IMUS);
    imu_counter++;
    
    // In a real implementation, you would:
    // 1. Check the interrupt pin: if (!gpio_get(bno->int_pin)) return;
    // 2. Read data packets from BNO08x via SPI
    // 3. Parse packets using sh2 library functions
    // 4. Push parsed sensor events to the queue
}

// Get sensor event from queue
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value) {
    if (!bno || !value) return false;
    
    return bno08x_event_queue_pop(&bno->event_queue, value);
}

// Multi-IMU management functions
bool multi_bno08x_init(Multi_BNO08x_t *multi) {
    if (!multi) return false;
    
    printf("Initializing Multi-BNO08x system...\n");
    memset(multi, 0, sizeof(Multi_BNO08x_t));
    multi->imu_count = 0;
    
    return true;
}

bool multi_bno08x_add_spi_imu(Multi_BNO08x_t *multi, uint8_t index, spi_inst_t *spi_port,
                              uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                              uint8_t cs_pin, uint8_t int_pin, uint8_t reset_pin, uint32_t spi_speed) {
    
    if (!multi || index >= MAX_BNO08X_IMUS) {
        printf("ERROR: Invalid parameters - multi=%p, index=%d (max=%d)\n", 
               multi, index, MAX_BNO08X_IMUS);
        return false;
    }
    
    printf("Adding IMU %d: CS=%d, INT=%d, RST=%d\n", index, cs_pin, int_pin, reset_pin);
    
    bool success = pico_bno08x_begin_spi(&multi->imus[index], spi_port, 
                                         miso_pin, mosi_pin, sck_pin, 
                                         cs_pin, int_pin, reset_pin, spi_speed);
    
    if (success) {
        multi->imus[index].initialized = true;
        if (index >= multi->imu_count) {
            multi->imu_count = index + 1;
        }
        printf("✓ IMU %d initialized successfully\n", index);
    } else {
        printf("✗ Failed to initialize IMU %d\n", index);
    }
    
    return success;
}

void multi_bno08x_service_all(Multi_BNO08x_t *multi) {
    if (!multi) return;
    
    for (uint8_t i = 0; i < multi->imu_count; i++) {
        if (multi->imus[i].initialized) {
            pico_bno08x_service(&multi->imus[i]);
        }
    }
}

bool multi_bno08x_get_sensor_event(Multi_BNO08x_t *multi, uint8_t imu_index, sh2_SensorValue_t *value) {
    if (!multi || !value || imu_index >= multi->imu_count) {
        return false;
    }
    
    if (!multi->imus[imu_index].initialized) {
        return false;
    }
    
    return pico_bno08x_get_sensor_event(&multi->imus[imu_index], value);
}

bool multi_bno08x_is_imu_initialized(Multi_BNO08x_t *multi, uint8_t imu_index) {
    if (!multi || imu_index >= multi->imu_count) return false;
    
    return multi->imus[imu_index].initialized;
}

uint8_t multi_bno08x_get_imu_count(Multi_BNO08x_t *multi) {
    if (!multi) return 0;
    
    return multi->imu_count;
}

void multi_bno08x_enable_all_reports(Multi_BNO08x_t *multi, uint8_t sensorId, uint32_t interval_us) {
    if (!multi) return;
    
    printf("Enabling sensor reports: ID=0x%02X, interval=%lu us\n", sensorId, interval_us);
    
    }
}
