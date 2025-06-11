/*!
 *  @file Pico_BNO08x_multi.c
 *
 *  SPI‑only driver for multiple BNO08x on Raspberry Pi Pico
 *  Based on the old single‑IMU code you verified, minus interface_type.
 */

#include "Pico_BNO08x.h"
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/spi.h"

// Global pointer to whichever IMU is “active” for HAL callbacks
static Pico_BNO08x_t *current_bno = NULL;
static sh2_SensorValue_t *sensor_value = NULL;

// Forward declarations
static bool     pico_bno08x_init_common(Pico_BNO08x_t *bno);
static void     hardware_reset(Pico_BNO08x_t *bno);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void     hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void     sensor_handler(void *cookie, sh2_SensorEvent_t *event);

// SPI HAL
static bool     spi_hal_wait_for_int(Pico_BNO08x_t *bno);
static int      spi_hal_open(sh2_Hal_t *self);
static void     spi_hal_close(sh2_Hal_t *self);
static int      spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us);
static int      spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len);


// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------

// Call this first on each IMU to make it “current”
void pico_bno08x_set_active(Pico_BNO08x_t *bno) {
    current_bno = bno;
}

bool pico_bno08x_init(Pico_BNO08x_t *bno, int8_t reset_pin, uint8_t instance_id) {
    if (!bno) return false;
    memset(bno, 0, sizeof(*bno));
    bno->reset_pin      = reset_pin;
    bno->int_pin        = -1;
    bno->cs_pin         = -1;
    bno->spi_speed      = 1000000;
    bno->reset_occurred = false;
    bno->hal.getTimeUs  = hal_get_time_us;
    return true;
}

bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno,
                           spi_inst_t *spi,
                           uint8_t miso, uint8_t mosi, uint8_t sck,
                           uint8_t cs,   uint8_t irq,
                           uint32_t speed) {
    if (!bno || !spi) return false;

    // Make this the active IMU
    current_bno = bno;

    bno->spi_port  = spi;
    bno->miso_pin  = miso;
    bno->mosi_pin  = mosi;
    bno->sck_pin   = sck;
    bno->cs_pin    = cs;
    bno->int_pin   = irq;
    bno->spi_speed = speed;

    // Initialize SPI peripheral once per bus
    static bool spi0done=false, spi1done=false;
    if (spi==spi0 && !spi0done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi0, speed);
        spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi0done=true;
    } else if (spi==spi1 && !spi1done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi1, speed);
        spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi1done=true;
    }

    // Chip‐select pin
    gpio_init(cs);
    gpio_set_dir(cs, GPIO_OUT);
    gpio_put(cs, 1);

    // Interrupt pin
    gpio_init(irq);
    gpio_set_dir(irq, GPIO_IN);
    gpio_pull_up(irq);

    // Hook up SPI HAL callbacks
    bno->hal.open  = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read  = spi_hal_read;
    bno->hal.write = spi_hal_write;

    return pico_bno08x_init_common(bno);
}

void pico_bno08x_hardware_reset(Pico_BNO08x_t *bno) {
    hardware_reset(bno);
}

bool pico_bno08x_was_reset(Pico_BNO08x_t *bno) {
    if (!bno) return false;
    bool r = bno->reset_occurred;
    bno->reset_occurred = false;
    return r;
}

bool pico_bno08x_enable_report(Pico_BNO08x_t *bno, sh2_SensorId_t id, uint32_t interval_us) {
    if (!bno) return false;
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled  = false,
        .wakeupEnabled             = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled           = false,
        .changeSensitivity         = 0,
        .batchInterval_us          = 0,
        .sensorSpecific            = 0,
        .reportInterval_us         = interval_us
    };
    return (sh2_setSensorConfig(id, &cfg) == SH2_OK);
}

bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *val) {
    // make sure HAL callbacks reference the right IMU
    current_bno = bno;
    sensor_value = val;
    val->timestamp = 0;
    sh2_service();
    sensor_value = NULL;
    return (val->timestamp != 0 || val->sensorId == SH2_GYRO_INTEGRATED_RV);
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    current_bno = bno;
    sh2_service();
}


// -----------------------------------------------------------------------------
// INTERNAL
// -----------------------------------------------------------------------------

static bool pico_bno08x_init_common(Pico_BNO08x_t *bno) {
    // hardware reset
    hardware_reset(bno);

    // open SH2: pass cookie=NULL, we use global current_bno
    int st = sh2_open(&bno->hal, hal_callback, NULL);
    if (st != SH2_OK) return false;

    // product IDs
    memset(&bno->prodIds, 0, sizeof(bno->prodIds));
    sh2_getProdIds(&bno->prodIds);

    // register sensor event callback (uses global sensor_value)
    sh2_setSensorCallback(sensor_handler, NULL);

    return true;
}

static void hardware_reset(Pico_BNO08x_t *bno) {
    if (!bno || bno->reset_pin<0) return;
    gpio_init(bno->reset_pin);
    gpio_set_dir(bno->reset_pin, GPIO_OUT);
    gpio_put(bno->reset_pin, 1);
    sleep_ms(10);
    gpio_put(bno->reset_pin, 0);
    sleep_ms(10);
    gpio_put(bno->reset_pin, 1);
    sleep_ms(20);
}

static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    (void)self;
    return time_us_32();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *e) {
    (void)cookie;
    if (!current_bno) return;
    if (e->eventId == SH2_RESET) current_bno->reset_occurred = true;
}

// SPI HAL implementations
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    if (!bno) return false;
    for (int i=0; i<500; i++) {
        if (!gpio_get(bno->int_pin)) return true;
        sleep_ms(1);
    }
    hardware_reset(bno);
    return false;
}

static int spi_hal_open(sh2_Hal_t *self) {
    (void)self;
    if (!current_bno) return -1;
    return spi_hal_wait_for_int(current_bno) ? 0 : -1;
}

static void spi_hal_close(sh2_Hal_t *self) {
    (void)self;
}

static int spi_hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    (void)self;
    if (!current_bno||!buf) return 0;
    if (!spi_hal_wait_for_int(current_bno)) return 0;

    uint8_t hdr[4];
    gpio_put(current_bno->cs_pin,0);
    spi_read_blocking(current_bno->spi_port,0x00,hdr,4);
    gpio_put(current_bno->cs_pin,1);

    uint16_t sz = ((uint16_t)hdr[0]|((uint16_t)hdr[1]<<8))&~0x8000;
    if (!sz||sz>len) return 0;

    if (!spi_hal_wait_for_int(current_bno)) return 0;
    gpio_put(current_bno->cs_pin,0);
    spi_read_blocking(current_bno->spi_port,0x00,buf,sz);
    gpio_put(current_bno->cs_pin,1);

    if(t_us)*t_us=time_us_32();
    return sz;
}

static int spi_hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    (void)self;
    if (!current_bno||!buf) return 0;
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    gpio_put(current_bno->cs_pin,0);
    spi_write_blocking(current_bno->spi_port,buf,len);
    gpio_put(current_bno->cs_pin,1);
    return len;
}

// Sensor handler
static void sensor_handler(void *cookie, sh2_SensorEvent_t *event){
    (void)cookie;
    if (!sensor_value) return;
    if (sh2_decodeSensorEvent(sensor_value,event)!=SH2_OK)
        sensor_value->timestamp=0;
}
