#ifndef PICO_BNO08X_H
#define PICO_BNO08X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif
// Extern global debug control flag
extern bool debug_suppressed;
/**
 * @brief BNO08x device state and configuration for a single instance
 *
 * Each BNO08x instance holds its own SPI config, SH2 instance, interrupt, and
 * pending sensor data, allowing true multi-IMU use.
 */
typedef struct {
    // Hardware SPI config
    spi_inst_t *spi_port;     ///< SPI port (e.g., spi0 or spi1)
    uint8_t cs_pin;           ///< Chip select pin (active low)
    uint8_t int_pin;          ///< Interrupt pin (active low)
    uint8_t reset_pin;        ///< Reset pin (active low)
    uint32_t spi_frequency;   ///< SPI clock speed in Hz
    int instance_id;          ///< Optional user-specified instance ID
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    
    // SH2 instance (each IMU gets its own)
    sh2_Instance_t *sh2_instance;  ///< Dedicated SH2 instance for this IMU
    sh2_Hal_t hal;                 ///< HAL interface for this instance

    // Sensor value buffer
    sh2_SensorValue_t sensor_value;      ///< Most recent sensor data
    sh2_SensorValue_t *pending_value;    ///< Pointer to data ready to be read

    // State flags
    bool initialized;
    bool spi_begun;
    bool has_reset;

    // Timing
    uint32_t last_reset_time_us;
    uint32_t last_service_time_us;
    
} Pico_BNO08x_t;

// Function prototypes

/**
 * @brief Initialize BNO08x instance (creates dedicated SH2 instance)
 * @param bno Pointer to BNO08x instance structure
 * @param reset_pin GPIO pin for reset (active low)
 * @param instance_id Unique ID for this instance (for debugging)
 * @return true if successful, false otherwise
 */
bool pico_bno08x_init(Pico_BNO08x_t *bno, int reset_pin, int instance_id);

/**
 * @brief Start SPI communication and register SH2 callbacks
 * @param bno Pointer to BNO08x instance
 * @param spi_port SPI port to use (spi0 or spi1)
 * @param miso_pin GPIO pin for MISO
 * @param mosi_pin GPIO pin for MOSI  
 * @param sck_pin GPIO pin for SCK
 * @param cs_pin GPIO pin for CS (chip select)
 * @param int_pin GPIO pin for interrupt
 * @param frequency SPI frequency in Hz
 * @return true if successful, false otherwise
 */
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno, spi_inst_t *spi_port,
                           uint8_t miso_pin, uint8_t mosi_pin, uint8_t sck_pin,
                           uint8_t cs_pin, uint8_t int_pin, uint32_t frequency);

/**
 * @brief Service function - must be called regularly for each IMU
 * @param bno Pointer to BNO08x instance
 */
void pico_bno08x_service(Pico_BNO08x_t *bno);

/**
 * @brief Enabl52%] Building C object src/CMakeFiles/original_test.dir/3IMUmain.c.o
[ 52%] Linking CXX executable original_test.elf
/usr/lib/gcc/arm-none-eabi/10.3.1/../../../arm-none-eabi/bin/ld: CMakeFiles/original_test.dir/Pico_BNO08x_multi.c.o: in function `spi_hal_close':
/app/src/Pico_BNO08x_multi.c:167: undefined reference to `debug_suppressed'
/usr/lib/gcc/arm-none-eabi/10.3.1/../../../arm-none-eabi/bin/ld: CMakeFiles/original_test.dir/Pico_BNO08x_multi.c.o: in function `spi_hal_open':
/app/src/Pico_BNO08x_multi.c:163: undefined reference to `debug_suppressed'
/usr/lib/gcc/arm-none-eabi/10.3.1/../../../arm-none-eabi/bin/ld: CMakeFiles/original_test.dir/Pico_BNO08x_multi.c.o: in function `hal_callback':
/app/src/Pico_BNO08x_multi.c:149: undefined reference to `debug_suppressed'
/usr/lib/gcc/arm-none-eabi/10.3.1/../../../arm-none-eabi/bin/ld: CMakeFiles/original_test.dir/Pico_BNO08x_multi.c.o: in function `spi_hal_read':
/app/src/Pico_BNO08x_multi.c:186: undefined reference to `debug_suppressed'
/usr/lib/gcc/arm-none-eabi/10.3.1/../../../arm-none-eabi/bin/ld: CMakeFiles/original_test.dir/Pico_BNO08x_multi.c.o: in function `spi_hal_write':
/app/src/Pico_BNO08x_multi.c:198: undefined reference to `debug_suppressed'
/usr/lib/gcc/arm-none-eabi/10.3.1/../../../arm-none-eabi/bin/ld: CMakeFiles/original_test.dir/Pico_BNO08x_multi.c.o:/app/src/Pico_BNO08x_multi.c:93: more undefined references to `debug_suppressed' follow
collect2: error: ld returned 1 exit status
make[2]: *** [src/CMakeFiles/original_test.dir/build.make:1465: src/original_test.elf] Error 1
make[1]: *** [CMakeFiles/Makefile2:2063: src/CMakeFiles/original_test.dir/all] Error 2
make: *** [Makefile:91: all] Error 2
e a sensor report for this IMU instance
 * @param bno Pointer to BNO08x instance
 * @param sensor_id Sensor type to enable
 * @param interval_us Report interval in microseconds
 * @return true if successful, false otherwise
 */
bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id, uint32_t interval_us);

/**
 * @brief Disable a sensor report for this IMU instance
 * @param bno Pointer to BNO08x instance
 * @param sensor_id Sensor type to disable
 * @return true if successful, false otherwise
 */
bool pico_bno08x_disable_report(Pico_BNO08x_t *bno, sh2_SensorId_t sensor_id);

/**
 * @brief Get sensor data from this IMU instance
 * @param bno Pointer to BNO08x instance
 * @param value Pointer to structure to receive sensor data
 * @return true if data was available, false otherwise
 */
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *value);

/**
 * @brief Check if data is available from this IMU instance
 * @param bno Pointer to BNO08x instance
 * @return true if data is available, false otherwise
 */
bool pico_bno08x_data_available(Pico_BNO08x_t *bno);

/**
 * @brief Perform hardware reset on this IMU instance
 * @param bno Pointer to BNO08x instance
 */
void pico_bno08x_reset(Pico_BNO08x_t *bno);

/**
 * @brief Perform software reset on this IMU instance
 * @param bno Pointer to BNO08x instance
 * @return true if successful, false otherwise
 */
bool pico_bno08x_soft_reset(Pico_BNO08x_t *bno);

/**
 * @brief Cleanup and destroy BNO08x instance
 * @param bno Pointer to BNO08x instance
 */
void pico_bno08x_destroy(Pico_BNO08x_t *bno);

#ifdef __cplusplus
}
#endif

#endif // PICO_BNO08X_H