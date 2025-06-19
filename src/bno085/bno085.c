#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/timer.h>

#include <sh2/sh2.h>
#include <sh2/sh2_err.h>

#include "bno085.h"

#ifndef SPIDEV_FREQUENCY
#define SPIDEV_FREQUENCY 30000000U
#endif

#ifndef BNO_TIMEOUT
#define BNO_TIMEOUT 1000000000UL
#endif

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)((char *)__mptr - offsetof(type,member));})

static int bno085_spi_open(sh2_Hal_t *self) {
	struct bno085* bno = container_of(self, struct bno085, sh2_hal);
	spi_inst_t* spidev = bno->spidev;
	unsigned timeout = BNO_TIMEOUT;

	/* set up SPI device */
	spi_init(spidev, SPIDEV_FREQUENCY);
	spi_set_format(spidev, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

	/* set up pins */
	gpio_set_function(bno->spidata.miso, GPIO_FUNC_SPI);
	gpio_set_function(bno->spidata.mosi, GPIO_FUNC_SPI);
	gpio_set_function(bno->spidata.sclk, GPIO_FUNC_SPI);

	gpio_init(bno->reset);
	gpio_init(bno->spidata.cs);
	gpio_init(bno->interrupt);

	/* set default values on the lines */
	gpio_set_dir_out_masked((1u << bno->reset) | (1u << bno->spidata.cs));	

	/* perform a reset cycle on the sensor hub to ensure communications start
	 * from a known state */
	gpio_put(bno->reset, 0);
	/* wait for device to be responsive, figure 1-22 */
	while(!gpio_get(bno->interrupt) && timeout) {
		timeout--;
	}
	gpio_put(bno->reset, 1);

	if(timeout) { bno->initialized = true; }
	return timeout ? SH2_OK : SH2_ERR_TIMEOUT;;
}

static void bno085_spi_close(sh2_Hal_t* self) {
	struct bno085* bno = container_of(self, struct bno085, sh2_hal);

	/* completes communication with the sensor */
	/* TODO: ??? */

	/* put the device in reset */
	gpio_put(bno->reset, 0);

	/* deinitialize peripherals or hardware resources that were used */
	spi_deinit(bno->spidev);

	bno->initialized = false;
}

static int bno085_spi_read(sh2_Hal_t* self, uint8_t* pBuffer,
					 unsigned len, uint32_t* t_us) {
	/* if the HAL has received a full SHTP transfer, this function should load
	 * the data into pBuffer, set the timestamp to the time the interrupt was
	 * detected, and return the non-zero length of data in this transfer */

	/* if the HAL has not received a full SHTP transfer, this function should
	 * return 0 */
	struct bno085* bno = container_of(self, struct bno085, sh2_hal);
	spi_inst_t* spidev = bno->spidev;
	unsigned tlen;

	/* TODO: this code may be incomplet or incorrekt or both, as it is based
	 * off Adafruit's shoddy library; hic sunt dracones */
	gpio_put(bno->spidata.cs, 0);
	spi_read_blocking(spidev, 0, pBuffer, 4);
	tlen = (unsigned)pBuffer[0] + ((unsigned)pBuffer[1] << 8);
	spi_read_blocking(spidev, 0, pBuffer, tlen);
	gpio_put(bno->spidata.cs, 1);
	return (int)tlen;	/* hooray for possible UB :( */
}

static int bno085_spi_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len) {
	/* if the device isn't ready to receive data, this function can return 0
	 * without performing the transmit function */

	/* if the transmission can be started, this function needs to copy the data
	 * from pBuffer and return the number of bytes accepted; it need not block;
	 * the actual transmission of the data can continue after this function
	 * returns */
	struct bno085* bno = container_of(self, struct bno085, sh2_hal);
	spi_inst_t* spidev = bno->spidev;

	/* set CS low */
	gpio_put(bno->spidata.cs, 0);
	/* do a blocking write anyway */
	spi_write_blocking(spidev, pBuffer, len);
	/* set CS high */
	gpio_put(bno->spidata.cs, 1);
	return (int)len;	/* possible UB if len > INT_MAX */
}

static uint32_t bno085_spi_get_time_us(sh2_Hal_t* self) {
	(void)self;
	return time_us_32();
}

static void bno085_event_callback(void* cookie, sh2_AsyncEvent_t* pEvent) {
	struct bno085* bno = (struct bno085*)cookie;
	if(pEvent->eventId == SH2_RESET) {
		bno->initialized = true;
	}
}

struct bno085*
bno085_init(struct bno085* bno) {

	/* pointers */
	bno->read_ptr = bno->write_ptr = 0;

	/* SH2 HAL callbaks */
	bno->sh2_hal.open = bno085_spi_open;
	bno->sh2_hal.close = bno085_spi_close;
	bno->sh2_hal.read = bno085_spi_read;
	bno->sh2_hal.write = bno085_spi_write;
	bno->sh2_hal.getTimeUs = bno085_spi_get_time_us;

	bno->initialized = false;

	return sh2_open(&bno->sh2_hal, bno085_event_callback, bno)
			? (struct bno085*)NULL : bno;
}

/* vim: set tw=79 tabstop=4: */
