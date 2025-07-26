#include "Pico_BNO085_I2C.h"
#include "pico/stdlib.h"
#include <string.h>
#include "sh2M.h"
#include "sh2_err.h"



static BNO085_I2C_t *_active = NULL;
static sh2_Hal_t     _hal = {0};

static uint32_t _hal_getTimeUs(sh2_Hal_t *self) {
    (void)self;
    return to_us_since_boot(get_absolute_time());
}
static int _hal_open(sh2_Hal_t *self) { (void)self; return SH2_OK; }
static void _hal_close(sh2_Hal_t *self) { (void)self; }

static int _hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    if (!_active) return SH2_ERR;
    bno085_mux_select(_active->i2c_port, _active->mux_channel);
    int r = i2c_read_blocking(_active->i2c_port, BNO085_I2C_ADDR, buf, len, false);
    if (r < 0) return SH2_ERR_IO;
    if (t_us) *t_us = to_us_since_boot(get_absolute_time());
    return SH2_OK;
}
static int _hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    if (!_active) return SH2_ERR;
    bno085_mux_select(_active->i2c_port, _active->mux_channel);
    int r = i2c_write_blocking(_active->i2c_port, BNO085_I2C_ADDR, buf, len, false);
    return (r < 0) ? SH2_ERR_IO : SH2_OK;
}

bool bno085_mux_select(i2c_inst_t *i2c_port, uint8_t channel) {
    uint8_t cmd = 1u << channel;
    return i2c_write_blocking(i2c_port, MUX_I2C_ADDR, &cmd, 1, false) == 1;
}

bool bno085_init(BNO085_I2C_t *imu, i2c_inst_t *i2c_port, uint8_t ch) {
    imu->i2c_port = i2c_port;
    imu->mux_channel = ch;
    imu->has_event = false;
    _active = imu;

    memset(&_hal, 0, sizeof(_hal));
    _hal.getTimeUs = _hal_getTimeUs;
    _hal.open      = _hal_open;
    _hal.close     = _hal_close;
    _hal.read      = _hal_read;
    _hal.write     = _hal_write;

    if (!bno085_mux_select(i2c_port, ch)) return false;
    imu->sh2 = sh2_createInstance();
    if (!imu->sh2) return false;
    if (sh2_openInstance(imu->sh2, &_hal, NULL, imu) != SH2_OK) return false;

    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = 20000;
    if (sh2_setSensorConfigInstance(imu->sh2, SH2_ROTATION_VECTOR, &cfg) != SH2_OK)
        return false;

    return true;
}

bool bno085_read_quat(BNO085_I2C_t *imu, float *w, float *x, float *y, float *z) {
    _active = imu;
    sh2_serviceInstance(imu->sh2);

    sh2_SensorEvent_t e;
    if (sh2_getSensorEventInstance(imu->sh2, &e) == SH2_OK) {
        if (e.reportId != SH2_ROTATION_VECTOR) return false;
        imu->last_event = e;
        imu->has_event = true;
    }

    if (!imu->has_event) return false;
    const uint8_t *r = imu->last_event.report;
    // rotation vector is 4×16-bit little‑endian fixed‑point Q30 (Bosch spec)
    uint32_t rawReal = (uint32_t)r[4] | ((uint32_t)r[5] << 8) | ((uint32_t)r[6] << 16) | ((uint32_t)r[7] << 24);
    uint32_t rawI    = (uint32_t)r[8] | ((uint32_t)r[9] << 8) | ((uint32_t)r[10] << 16) | ((uint32_t)r[11] << 24);
    uint32_t rawJ    = (uint32_t)r[12] | ((uint32_t)r[13] << 8) | ((uint32_t)r[14] << 16) | ((uint32_t)r[15] << 24);
    uint32_t rawK    = (uint32_t)r[16] | ((uint32_t)r[17] << 8) | ((uint32_t)r[18] << 16) | ((uint32_t)r[19] << 24);

    *w = (int32_t)rawReal * (1.0f / (1LL << 30));
    *x = (int32_t)rawI    * (1.0f / (1LL << 30));
    *y = (int32_t)rawJ    * (1.0f / (1LL << 30));
    *z = (int32_t)rawK    * (1.0f / (1LL << 30));

    imu->has_event = false;
    return true;
}
