/**
 * BNO08x Example for Raspberry Pi Pico
 * 
 * This example demonstrates how to use the BNO08x IMU sensor with the Raspberry Pi Pico
 * Supports I2C, SPI, and UART interfaces
 */

#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/spi.h>

#include <bno085/bno085.h>

struct bno085 imu[2] = {
		[0] = {
			.spidev = spi0,
			.spidata = { .miso = 2, .mosi = 3, .sclk = 4, .cs = 5 },
			.reset = 10,
			.interrupt = 11
		},
		[1] = {
			.spidev = spi1,
			.spidata = { .miso = 6, .mosi = 7, .sclk = 8, .cs = 9 },
			.reset = 12,
			.interrupt = 13
		}
	};

int main() {
    stdio_init_all();

	if(!bno085_init(&imu[0])) {
		printf("error initializing IMU0\n");
		/* TODO: do something better */
	}

	if(!bno085_init(&imu[1])) {
		printf("error initializing IMU1\n");
	}
    
    return 0;
}
