#include <cstdio>
#include <cstdlib>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "../include/TransferValuesToArduino.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))


static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.2";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

void ByteTranslation(unsigned char x_separated[5], int x)
{
	const unsigned int char_size = 8;
	unsigned int hash = 0;
	for (int i = 0; i < 4; i++)
	{
		x_separated[i] = ((x >> char_size * i) & 0xFF);
		hash += x_separated[i];
	}
	x_separated[4] = (unsigned char)(hash % 256);
}

static void transfer(int fd, int angle, unsigned char order)
{
	int ret;
	unsigned char angle_transe[5];
	ByteTranslation(angle_transe, angle);

	uint8_t tx[] = {
		angle_transe[0], angle_transe[1], angle_transe[2], angle_transe[3], angle_transe[4],
		order, order,
		0x0A};
	uint8_t rx[ARRAY_SIZE(tx)] = {
		0,
	};
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.speed_hz = speed,
		.delay_usecs = delay,
		.bits_per_word = bits};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
}

void TransferValuesToArduino(int angle, unsigned char order)
{
	int fd = open(device, O_RDWR);
	if (fd < 0)
	{
		pabort("can't open device");
	}

	/*
	 * spi mode
	 */
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");


	transfer(fd, angle, order);

	close(fd);
}
