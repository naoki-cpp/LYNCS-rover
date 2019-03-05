#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "../include/TransferValuesToArduino.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

using namespace std;
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

TransferValuesToArduino::TransferValuesToArduino()
	: bits(8),
	  speed(500000)
{
}

TransferValuesToArduino::~TransferValuesToArduino()
{
	close(fd_);
}
int TransferValuesToArduino::Transfer(int angle, unsigned char order)
{
	int ret;
	unsigned char angle_trans[5];
	ByteTranslation(angle_trans, angle);

	uint8_t tx[] = {
		angle_trans[0], angle_trans[1], angle_trans[2], angle_trans[3], angle_trans[4],
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

	ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
	{
		cerr << "can't send spi message" << endl;
		return ret;
	}
	return 0;
}
int TransferValuesToArduino::Init()
{
	fd_ = open("/dev/spidev1.2", O_RDWR);
	if (fd_ < 0)
	{
		cerr << "can't open device" << endl;
		return -1;
	}else{
		cout << "successfully opend the device" << endl;
	}

	/*
	 * spi mode
	 */
	int ret = ioctl(fd_, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		cerr << "can't set spi mode" << endl;
		return -1;
	}

	ret = ioctl(fd_, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		cerr << "can't get spi mode" << endl;
		return -1;
	}

	/*
	 * bits per word
	 */
	ret = ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		cerr << "can't set bits per word" << endl;
		return -1;
	}

	ret = ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		cerr << "can't get bits per word" << endl;
		return -1;
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		cerr << "can't set max speed hz" << endl;
		return -1;
	}

	ret = ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		cerr << "can't get max speed hz" << endl;
		return -1;
	}

	return fd_;
}
