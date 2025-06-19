#ifndef BNO085_H__
#define BNO085_H__

#include <stdbool.h>
#include <stdint.h>

#include <hardware/spi.h>

#include <sh2/sh2.h>

struct spi_data {
	uint8_t miso, mosi, sclk, cs;
};

struct bno085 {
	spi_inst_t* spidev;
	struct spi_data spidata;
	sh2_Hal_t sh2_hal;
	bool initialized;
	unsigned read_ptr, write_ptr;
	uint8_t reset, interrupt;
};

struct bno085*
bno085_init(struct bno085* bno);

#endif

/* vim: set tw=79 tabstop=4: */
