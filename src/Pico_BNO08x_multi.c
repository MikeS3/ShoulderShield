/*
 * Pico_BNO08x_multi.c
 * SPI‑only driver for multiple BNO08x on Raspberry Pi Pico
 */

#include "Pico_BNO08x.h"
#include <string.h>

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)((char *)__mptr - offsetof(type,member));})

// “Active” instance for HAL callbacks
static Pico_BNO08x_t *current_bno = NULL;
static sh2_SensorValue_t *sensor_value = NULL;

// Forward decls
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

// — API —

// Switch the “active” IMU before any SH2 call
void pico_bno08x_set_active(Pico_BNO08x_t *bno) {
    current_bno = bno;
}

// Basic init (zero struct, set reset pin, HAL timer)
bool pico_bno08x_init(Pico_BNO08x_t *bno, int8_t reset_pin) {
    if (!bno) return false;
    memset(bno, 0, sizeof(*bno));
    bno->reset_pin     = reset_pin;
    bno->spi_speed     = 1000000;
    bno->reset_occurred= false;
    bno->hal.getTimeUs = hal_get_time_us;
    return true;
}

// SPI begin: sets pins, init bus once, hooks HAL callbacks
bool pico_bno08x_begin_spi(Pico_BNO08x_t *bno,
                           spi_inst_t *spi,
                           uint8_t miso, uint8_t mosi, uint8_t sck,
                           uint8_t cs,   uint8_t irq,
                           uint32_t speed) {
    if (!bno || !spi) return false;
    current_bno = bno;
    bno->spi_port = spi;
    bno->miso_pin = miso;
    bno->mosi_pin = mosi;
    bno->sck_pin  = sck;
    bno->cs_pin   = cs;
    bno->int_pin  = irq;
    bno->spi_speed= speed;

    static bool spi0done=false, spi1done=false;
    if (spi==spi0 && !spi0done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi0, speed);
        spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi0done = true;
    }
    if (spi==spi1 && !spi1done) {
        gpio_set_function(miso, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(sck,  GPIO_FUNC_SPI);
        spi_init(spi1, speed);
        spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi1done = true;
    }

    gpio_init(cs);    gpio_set_dir(cs, GPIO_OUT); gpio_put(cs,1);
    gpio_init(irq);   gpio_set_dir(irq,GPIO_IN);   gpio_pull_up(irq);

    bno->hal.open  = spi_hal_open;
    bno->hal.close = spi_hal_close;
    bno->hal.read  = spi_hal_read;
    bno->hal.write = spi_hal_write;
    bno->hal.getTimeUs = hal_get_time_us;
    return pico_bno08x_init_common(bno);
}

// Hardware reset (GPIO toggle)
static void hardware_reset(Pico_BNO08x_t *bno) {
    if (!bno || bno->reset_pin<0) return;
    gpio_init(bno->reset_pin);
    gpio_set_dir(bno->reset_pin, GPIO_OUT);
    gpio_put(bno->reset_pin, 1);
    //sleep_ms(10);
    gpio_put(bno->reset_pin, 0);
    //sleep_ms(10);
    gpio_put(bno->reset_pin, 1);
    //sleep_ms(20);
}

// Core SH2 init + callbacks
static bool pico_bno08x_init_common(Pico_BNO08x_t *bno) {
    hardware_reset(bno);
    if (sh2_open(&bno->hal, hal_callback, NULL) != SH2_OK) return false;
    memset(&bno->prodIds,0,sizeof(bno->prodIds));
    sh2_getProdIds(&bno->prodIds);
    sh2_setSensorCallback(sensor_handler, NULL);
    return true;
}

// Get one sensor report (rotvec, accel, etc.)
bool pico_bno08x_get_sensor_event(Pico_BNO08x_t *bno, sh2_SensorValue_t *val) {
    current_bno   = bno;
    sensor_value  = val;
    val->timestamp= 0;
    sh2_service();
    sensor_value  = NULL;
    return (val->timestamp!=0 || val->sensorId==SH2_GYRO_INTEGRATED_RV);
}

void pico_bno08x_service(Pico_BNO08x_t *bno) {
    current_bno = bno;
    sh2_service();
}

bool pico_bno08x_enable_report(Pico_BNO08x_t *bno,
                               sh2_SensorId_t id, uint32_t iu) {
    current_bno = bno;
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled  = false,
        .wakeupEnabled             = false,
        .changeSensitivityRelative = false,
        .alwaysOnEnabled           = false,
        .changeSensitivity         = 0,
        .batchInterval_us          = 0,
        .sensorSpecific            = 0,
        .reportInterval_us         = iu
    };
    return (sh2_setSensorConfig(id, &cfg)==SH2_OK);
}

// HAL callbacks
static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    (void)self;
    return time_us_32();
}
static void hal_callback(void *cookie, sh2_AsyncEvent_t *e) {
    (void)cookie;
    if (!current_bno) return;
    if (e->eventId==SH2_RESET) current_bno->reset_occurred = true;
}
static void sensor_handler(void *cookie, sh2_SensorEvent_t *evt){
    (void)cookie;
    if (!sensor_value) return;
    if (sh2_decodeSensorEvent(sensor_value,evt)!=SH2_OK)
        sensor_value->timestamp=0;
}

// SPI HAL
static bool spi_hal_wait_for_int(Pico_BNO08x_t *bno) {
    for(int i=0;i<500;i++){
        if (!gpio_get(bno->int_pin)) return true;
        sleep_ms(1);
    }
    hardware_reset(bno);
    return false;
}
static int spi_hal_open(sh2_Hal_t *self){
    (void)self;
    return SH2_OK;
}
static void spi_hal_close(sh2_Hal_t *self){ (void)self; }
static int spi_hal_read(sh2_Hal_t *self,uint8_t *buf,unsigned len,uint32_t *t_us){
    /*(void)self;
    if (!current_bno||!buf) return 0;
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    uint8_t hdr[4];
    gpio_put(current_bno->cs_pin,0);
    spi_read_blocking(current_bno->spi_port,0x00,hdr,4);
    gpio_put(current_bno->cs_pin,1);
    uint16_t sz = ((uint16_t)hdr[0]|((uint16_t)hdr[1]<<8)) & ~0x8000;
    if (!sz||sz>len) return 0;
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    gpio_put(current_bno->cs_pin,0);
    spi_read_blocking(current_bno->spi_port,0x00,buf,sz);
    gpio_put(current_bno->cs_pin,1);
    if(t_us) *t_us=time_us_32();
    return sz; */                   //old sh2 test code 
    
    Pico_BNO08x_t* bno;
    bno = container_of(self, Pico_BNO08x_t, hal);
    gpio_put(bno->cs_pin, 0);
    int ret = spi_read_blocking(bno->spi_port, 0x00, buf, len);
    gpio_put(bno->cs_pin, 1);
    return ret;
}
static int spi_hal_write(sh2_Hal_t *self,uint8_t *buf,unsigned len){
   /* (void)self;
    if (!current_bno||!buf) return 0;
    if (!spi_hal_wait_for_int(current_bno)) return 0;
    gpio_put(current_bno->cs_pin,0);
    spi_write_blocking(current_bno->spi_port,buf,len);
    gpio_put(current_bno->cs_pin,1);
    return len; */

    Pico_BNO08x_t* bno;
    bno = container_of(self, Pico_BNO08x_t, hal);
    gpio_put(bno->cs_pin, 0);
    int ret = spi_write_blocking(bno->spi_port, buf, len);
    gpio_put(bno->cs_pin, 1);
    return ret;
}
