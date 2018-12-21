/***************************************************************************
Modified BSD License
====================
Copyright © 2016, Andrei Vainik
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of the organization nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
This piece of code was combined from several sources
https://github.com/adafruit/Adafruit_BME280_Library
https://cdn-shop.adafruit.com/datasheets/BST-BME280_DS001-10.pdf
https://projects.drogon.net/raspberry-pi/wiringpi/i2c-library/
Compensation functions and altitude function originally from:
https://github.com/adafruit/Adafruit_BME280_Library/blob/master/Adafruit_BME280.cpp
***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************
****************************************************************************/

#include <stdio.h>
#include<stdint.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include "bme280.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int v;
double a1;
double a2;
double TIME;
double TIME2;
double TIME1;



static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

int geta() {

  int fd = wiringPiI2CSetup(BME280_ADDRESS);
  if(fd < 0) {
    printf("Device not found");
    return -1;
  }

  bme280_calib_data cal;
  readCalibrationData(fd, &cal);

  wiringPiI2CWriteReg8(fd, 0xf2, 0x01);   // humidity oversampling x 1
  wiringPiI2CWriteReg8(fd, 0xf4, 0x25);   // pressure and temperature oversampling x 1, mode normal

  bme280_raw_data raw;
  getRawData(fd, &raw);

  int32_t t_fine = getTemperatureCalibration(&cal, raw.temperature);
  //float t = compensateTemperature(t_fine); // C
  float p = compensatePressure(raw.pressure, &cal, t_fine) / 100; // hPa
  //float h = compensateHumidity(raw.humidity, &cal, t_fine);       // %
  float a = getAltitude(p);                         // meters

  //printf("{\"sensor\":\"bme280\", \"humidity\":%.2f, \"pressure\":%.2f,"
//   " \"temperature\":%.2f, \"altitude\":%.2f, \"timestamp\":%d}\n",
//    h, p, t, a, (int)time(NULL));

    a1=a;
    TIME1=clock();
    //TIME=(TIME1-TIME2)/CLOCKS_PER_SEC; //1000000
		TIME = 0.1;
    TIME2=TIME1;
    v=(int)((a1-a2)/TIME*10);
		//	printf("speed%d\n",v);
    a2=a1;


    return 0;


}

int32_t getTemperatureCalibration(bme280_calib_data *cal, int32_t adc_T) {
  int32_t var1  = ((((adc_T>>3) - ((int32_t)cal->dig_T1 <<1))) *
     ((int32_t)cal->dig_T2)) >> 11;

  int32_t var2  = (((((adc_T>>4) - ((int32_t)cal->dig_T1)) *
       ((adc_T>>4) - ((int32_t)cal->dig_T1))) >> 12) *
     ((int32_t)cal->dig_T3)) >> 14;

  return var1 + var2;
}

void readCalibrationData(int fd, bme280_calib_data *data) {
  data->dig_T1 = (uint16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_T1);
  data->dig_T2 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_T2);
  data->dig_T3 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_T3);

  data->dig_P1 = (uint16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P1);
  data->dig_P2 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P2);
  data->dig_P3 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P3);
  data->dig_P4 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P4);
  data->dig_P5 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P5);
  data->dig_P6 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P6);
  data->dig_P7 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P7);
  data->dig_P8 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P8);
  data->dig_P9 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_P9);

  data->dig_H1 = (uint8_t)wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H1);
  data->dig_H2 = (int16_t)wiringPiI2CReadReg16(fd, BME280_REGISTER_DIG_H2);
  data->dig_H3 = (uint8_t)wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H3);
  data->dig_H4 = (wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H4) << 4) | (wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H4+1) & 0xF);
  data->dig_H5 = (wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H5+1) << 4) | (wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H5) >> 4);
  data->dig_H6 = (int8_t)wiringPiI2CReadReg8(fd, BME280_REGISTER_DIG_H6);
}

float compensateTemperature(int32_t t_fine) {
  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}

float compensatePressure(int32_t adc_P, bme280_calib_data *cal, int32_t t_fine) {
  int64_t var1, var2, p;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)cal->dig_P6;
  var2 = var2 + ((var1*(int64_t)cal->dig_P5)<<17);
  var2 = var2 + (((int64_t)cal->dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)cal->dig_P3)>>8) +
    ((var1 * (int64_t)cal->dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)cal->dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)cal->dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)cal->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)cal->dig_P7)<<4);
  return (float)p/256;
}


float compensateHumidity(int32_t adc_H, bme280_calib_data *cal, int32_t t_fine) {
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal->dig_H4) << 20) -
      (((int32_t)cal->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
         (((((((v_x1_u32r * ((int32_t)cal->dig_H6)) >> 10) *
        (((v_x1_u32r * ((int32_t)cal->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
      ((int32_t)2097152)) * ((int32_t)cal->dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
           ((int32_t)cal->dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r>>12);
  return  h / 1024.0;
}

void getRawData(int fd, bme280_raw_data *raw) {
  wiringPiI2CWrite(fd, 0xf7);

  raw->pmsb = wiringPiI2CRead(fd);
  raw->plsb = wiringPiI2CRead(fd);
  raw->pxsb = wiringPiI2CRead(fd);

  raw->tmsb = wiringPiI2CRead(fd);
  raw->tlsb = wiringPiI2CRead(fd);
  raw->txsb = wiringPiI2CRead(fd);

  raw->hmsb = wiringPiI2CRead(fd);
  raw->hlsb = wiringPiI2CRead(fd);

  raw->temperature = 0;
  raw->temperature = (raw->temperature | raw->tmsb) << 8;
  raw->temperature = (raw->temperature | raw->tlsb) << 8;
  raw->temperature = (raw->temperature | raw->txsb) >> 4;

  raw->pressure = 0;
  raw->pressure = (raw->pressure | raw->pmsb) << 8;
  raw->pressure = (raw->pressure | raw->plsb) << 8;
  raw->pressure = (raw->pressure | raw->pxsb) >> 4;

  raw->humidity = 0;
  raw->humidity = (raw->humidity | raw->hmsb) << 8;
  raw->humidity = (raw->humidity | raw->hlsb);
}

float getAltitude(float pressure) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return 44330.0 * (1.0 - pow(pressure / MEAN_SEA_LEVEL_PRESSURE, 0.190294957));
}

static void transfer(int fd,int e,int f,int x,int y,int z,int x1,int y1,int z1,unsigned char bob,unsigned char coc)
{
	int ret;
	unsigned int a;
	unsigned int b;
	unsigned int xx;
	unsigned int yy;
	unsigned int zz;
	unsigned int xxx;
	unsigned int yyy;
	unsigned int zzz;
	unsigned int g;
	unsigned int h;
	unsigned int xx1;
	unsigned int yy1;
	unsigned int zz1;
	unsigned int xxx1;
	unsigned int yyy1;
	unsigned int zzz1;
	unsigned char c[4];
	unsigned char d[4];
	unsigned char xx2[4];
	unsigned char yy2[4];
	unsigned char zz2[4];
	unsigned char xxx2[4];
	unsigned char yyy2[4];
	unsigned char zzz2[4];
	a=(unsigned int)e;
	b=(unsigned int)f;
	xx=(unsigned int)x;
	yy=(unsigned int)y;
	zz=(unsigned int)z;
	xxx=(unsigned int)x1;
	yyy=(unsigned int)y1;
	zzz=(unsigned int)z1;
	c[0]=(unsigned char)(a%256);
	g=a/256;
	d[0]=(unsigned char)(b%256);
	h=b/256;
	xx2[0]=(unsigned char)(xx%256);
	xx1=xx/256;
	yy2[0]=(unsigned char)(yy%256);
	yy1=yy/256;
	zz2[0]=(unsigned char)(zz%256);
	zz1=zz/256;
	xxx2[0]=(unsigned char)(xxx%256);
	xxx1=xxx/256;
	yyy2[0]=(unsigned char)(yyy%256);
	yyy1=yyy/256;
	zzz2[0]=(unsigned char)(zzz%256);
	zzz1=zzz/256;
	c[1]=(unsigned char)(g%256);
	g=g/256;
	d[1]=(unsigned char)(h%256);
	h=h/256;

	xx2[1]=(unsigned char)(xx1%256);
	xx1=xx1/256;
	yy2[1]=(unsigned char)(yy1%256);
	yy1=yy1/256;
	zz2[1]=(unsigned char)(zz1%256);
	zz1=zz1/256;
	xxx2[1]=(unsigned char)(xxx1%256);
	xxx1=xxx1/256;
	yyy2[1]=(unsigned char)(yyy1%256);
	yyy1=yyy1/256;
	zzz2[1]=(unsigned char)(zzz1%256);
	zzz1=zzz1/256;

	c[2]=(unsigned char)(g%256);
	g=g/256;
	d[2]=(unsigned char)(h%256);
	h=h/256;
	xx2[2]=(unsigned char)(xx1%256);
	xx1=xx1/256;
	yy2[2]=(unsigned char)(yy1%256);
	yy1=yy1/256;
	zz2[2]=(unsigned char)(zz1%256);
	zz1=zz1/256;
	xxx2[2]=(unsigned char)(xxx1%256);
	xxx1=xxx1/256;
	yyy2[2]=(unsigned char)(yyy1%256);
	yyy1=yyy1/256;
	zzz2[2]=(unsigned char)(zzz1%256);
	zzz1=zzz1/256;
	c[3]=(unsigned char)(g%256);
	g=g/256;
	d[3]=(unsigned char)(h%256);
	h=h/256;
	xx2[3]=(unsigned char)(xx1%256);
	xx1=xx1/256;
	yy2[3]=(unsigned char)(yy1%256);
	yy1=yy1/256;
	zz2[3]=(unsigned char)(zz1%256);
	zz1=zz1/256;
	xxx2[3]=(unsigned char)(xxx1%256);
	xxx1=xxx1/256;
	yyy2[3]=(unsigned char)(yyy1%256);
	yyy1=yyy1/256;
	zzz2[3]=(unsigned char)(zzz1%256);
	zzz1=zzz1/256;
	uint8_t tx[] = {
		c[0],c[1],c[2],c[3],
		d[0],d[1],d[2],d[3],
		xx2[0],xx2[1],xx2[2],xx2[3],
		yy2[0],yy2[1],yy2[2],yy2[3],
		zz2[0],zz2[1],zz2[2],zz2[3],
		xxx2[0],xxx2[1],xxx2[2],xxx2[3],
		yyy2[0],yy2[1],yyy2[2],yyy2[3],
		zzz2[0],zzz2[1],zzz2[2],zzz2[3],
		bob,coc,
        0x0A
	};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

    /*
	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
    */
}
void trns(int j1,int j2,int j3,int j4,int j5,int j6,unsigned char s1,unsigned char s2)
{
	int ret = 0;
	int fd;



	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
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

    geta();



	transfer(fd,j1,j2,j3,j4,j5,j6,v,v,s1,s2);
  //	usleep(100000); //上も変えよう



	close(fd);


}
