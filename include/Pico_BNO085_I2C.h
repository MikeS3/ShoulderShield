#ifndef PICO_BNO085_I2C_H
#define PICO_BNO085_I2C_H

#include "hardware/i2c.h"
#include "sh2M.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BNO085_I2C_ADDR 0x4A
#define MUX_I2C_ADDR    0x70

typedef struct {
    i2c_inst_t     *i2c_port;
    uint8_t         mux_channel;
    sh2_Instance_t *sh2;                  // pointer to instance
    sh2_SensorEvent_t last_event;        // most recent event
    bool            has_event;
} BNO085_I2C_t;

bool bno085_mux_select(i2c_inst_t *port, uint8_t ch);
bool bno085_init(BNO085_I2C_t *imu, i2c_inst_t *i2c_port, uint8_t mux_channel);
bool bno085_read_quat(BNO085_I2C_t *imu, float *w, float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif
#endif
