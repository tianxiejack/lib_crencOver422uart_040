

#ifndef _SPIDEV_TRANS_H_
#define _SPIDEV_TRANS_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define Tranverse32(X) ((((unsigned int)(X)&0xff000000)>>24)|(((unsigned int)(X)&0x00ff0000)>>8)|(((unsigned int)(X)&0x0000ff00)<<8)|(((unsigned int)(X)&0x000000ff)<<24))
//Tranverse32 -----  switch 32bit data from Little-endian  to Big-endian

int spi_init(const char *device, uint8_t bits, uint32_t speed);
int spi_close(int fd);
int spi_transfer(int fd, struct spi_ioc_transfer *tr);
int spi_dataTransform1(unsigned char *dst, unsigned char *src, int srcNum);
int spi_dataTransform2(unsigned char *dst, unsigned char *src, int srcNum);

#endif

