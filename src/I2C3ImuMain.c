#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Pico_BNO085_I2C.h"

#define I2C_PORT   i2c1
#define SDA_PIN    2
#define SCL_PIN    3
#define NUM_IMUS   3

int main(void) {
    stdio_init_all();
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    sleep_ms(100);

    BNO085_I2C_t imus[NUM_IMUS];
    for (int i = 0; i < NUM_IMUS; i++) {
        if (!bno085_init(&imus[i], I2C_PORT, i)) {
            printf("IMU %d init FAILED\n", i);
            while (1) tight_loop_contents();
        }
        printf("IMU %d ready (mux ch=%d)\n", i, i);
    }

    absolute_time_t next = get_absolute_time();
    while (1) {
        next = delayed_by_ms(next, 20);
        float qw[NUM_IMUS], qx[NUM_IMUS], qy[NUM_IMUS], qz[NUM_IMUS];
        for (int i = 0; i < NUM_IMUS; i++) {
            if (!bno085_read_quat(&imus[i], &qw[i], &qx[i], &qy[i], &qz[i])) {
                qw[i]=qx[i]=qy[i]=qz[i]=0.0f;
            }
        }
        float t = to_ms_since_boot(get_absolute_time()) * 0.001f;
        printf("%.3f", t);
        for (int i = 0; i < NUM_IMUS; i++)
            printf(",%.3f,%.3f,%.3f,%.3f", qw[i], qx[i], qy[i], qz[i]);
        printf("\n");
        sleep_until(next);
    }
}
