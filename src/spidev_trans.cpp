#include "spidev_trans.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

//FILE *savefp=NULL; //save data before send to spi

static void pabort(const char *s)
{
	perror(s);
	abort();
}

int spi_transfer(int fd, struct spi_ioc_transfer *tr)
{
	int ret;
#if 0
	/****save data before send to spi*************/
   	 if(savefp==NULL)
    		savefp = fopen("app_trans.ioc","wb");
   	 fwrite((const void *)(tr->tx_buf), tr->len, 1, savefp);
   	 //fflush(savefp);
   	 //printf("\n------had finished one spi_ioc_transfer----\n");
	 /****save data before send to spi***end*******/
#endif
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), tr);
	//if (ret < 1)
	//	pabort("can't send spi message");

	return ret;
}

int spi_dataTransform1(unsigned char *dst, unsigned char *src, int srcNum)
{
	unsigned int *p_dst	= (unsigned int *)dst;
	unsigned short *p_src 	= (unsigned short *)src;
	int i, j, Excess_num;
/*
	if(srcNum%2)
	{
		src[srcNum] = 0xFF;
		srcNum += 1;
	}
*/
	int RunTimes = srcNum/sizeof(unsigned short)/8;
	Excess_num	 = (srcNum - RunTimes*sizeof(unsigned short)*8)/sizeof(unsigned short);

	for(i=0; i<RunTimes; i++)
	{
		p_dst[0] = Tranverse32( (unsigned int)((0x0150000 | p_src[0])<<4) );
		p_dst[1] = Tranverse32( (unsigned int)((0x0150000 | p_src[1])<<4) );
		p_dst[2] = Tranverse32( (unsigned int)((0x0150000 | p_src[2])<<4) );
		p_dst[3] = Tranverse32( (unsigned int)((0x0150000 | p_src[3])<<4) );
		p_dst[4] = Tranverse32( (unsigned int)((0x0150000 | p_src[4])<<4) );
		p_dst[5] = Tranverse32( (unsigned int)((0x0150000 | p_src[5])<<4) );
		p_dst[6] = Tranverse32( (unsigned int)((0x0150000 | p_src[6])<<4) );
		p_dst[7] = Tranverse32( (unsigned int)((0x0150000 | p_src[7])<<4) );

		p_dst += 8;
		p_src += 8;
	}

	for(j=0; j<Excess_num; j++)
	{
		p_dst[0] = Tranverse32( (unsigned int)((0x0150000 | p_src[0])<<4) );
		p_dst += 1;
		p_src += 1;
	}
	return (srcNum)*2;

}

int spi_dataTransform2(unsigned char *dst, unsigned char *src, int srcNum)
{
	unsigned int *p_dst	= (unsigned int *)dst;
	unsigned int *p_src 	= (unsigned int *)src;
	int i, j, Excess_num;

	int RunTimes = srcNum/4;

	while(RunTimes)
	{
		//p_dst[0] = Tranverse32( (unsigned int)(p_src[0]) );
		p_dst[0] = (unsigned int)(p_src[0]);
		p_dst += 1;
		p_src += 1;
		--RunTimes;
	}	
	return (srcNum);	

}

#if 0
static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}
#endif

int spi_init(const char *device, uint8_t bits, uint32_t speed)
{
	int ret = 0, fd;
	uint8_t mode=0;

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

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	return fd;
}

int spi_close(int fd)
{
	int status = close(fd);
	if(status != 0)
		printf("close spi port fd[%d] failed\n", fd);

	return status;
}
