#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <osa.h>
#include <osa_tsk.h>
#include <osa_buf.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "gpio_rdwr.h"
#include "spidev_trans.h"
#include "sync422_trans.h"
#include "vidScheduler.h"

using namespace cr_osa;

#define DEBUGMODE	0
#define DEBUGPRI	0
#define SPIDEVON	1

#define DATAFORMAT1	1
#define DATAFORMAT2	2
#define DATAFORMAT	DATAFORMAT2

#define TIME_OUT	1000

// for PROJ_AXGS040
#define SYNC422_PORTNUM		1	// portA
#define RING_VIDEO_TV	0
#define RING_VIDEO_FR	1
#define RING_VIDEO_BUFLEN	0x080000	// 512KB
#define ENC_VIDEO_BUFMAX	0x040000	// 256KB

typedef struct
{
	 unsigned char sync[6];
	 unsigned char pktsize[4];
	 unsigned char dtype[2];
	 unsigned short transno;
	 unsigned char res[2];
}ENC_EVENTHEADER;

typedef struct
{
	 unsigned char syncHead[2];
	 unsigned char cntno;
	 unsigned char data[252];
	 unsigned char syncEnd;
}ENC_USR54HEADER;
// for PROJ_AXGS040 end

typedef struct _Sync422_TransObj {
	int fd;
	OSA_ThrHndl tskHndl;
	Bool tskLoop;
	Bool istskStopDone;
	OSA_BufHndl ringQue;
	OSA_BufCreate ringCreate;

	int spiuart;	// SPI_PORT_A or SPI_PORT_B
	Uint8 *tx_buf;
	//Uint8 *rx_buf;
	Uint8 *data_buf;
	struct spi_ioc_transfer spiTrans;
	Uint8 packet_tail[8];

	ENC_EVENTHEADER DataHead;
	ENC_USR54HEADER UsrHead;
	volatile int dataClock;	// SYNC422_CLOCK_2M or 4M or 8M
	volatile int dataPause;
}Sync422_TransObj;

typedef struct
{
    int chMask;
    int chfps[2];
    int pktPiece;
} Scheduler_arg;

////////////////////////////////
// sync422 part
#define SYNC422_CLOCK_2M			0
#define SYNC422_CLOCK_4M			1
#define SYNC422_CLOCK_8M			2

#define SYNC422_FIFO_UPLIMIT_A		(25000) //max less than 32000
#define SYNC422_FIFO_DNLIMIT_A		(10000) //

////////////////////////////////
// spi part
#define SPI_BUFFER_SIZE				(4096)

#define SPI_IOC_MAGIC			'k'
#define SPI_RD_IOC_SET		_IOR(SPI_IOC_MAGIC, 6, __u32)
#define SPI_WR_IOC_SET		_IOW(SPI_IOC_MAGIC, 6, __u32)
typedef enum
{
	FPGA_RD_VESRSION = 0x01,
	FPGA_RD_FIFO_STATSIZE,

	FPGA_WR_FIFO_UPPER = 0x01,
	FPGA_WR_FIFO_FLOOR,
	FPGA_WR_FIFO_RESET,
	FPGA_WR_CLOCK,
}SPI_CMD_CUSTOM;

typedef struct _spi_cmd_arg_ {
	unsigned int io_addr;	// SPI_CMD_CUSTOM
	unsigned int io_buffer;
}spi_cmd_arg;

// SPI_PORT_A
#define spidevA  "/dev/spidev3.0"

////////////////////////////////
// Local Global Variables
static int ibInit=0;
static int iChangeSpeed[SYNC422_PORTNUM]={0};
static Sync422_TransObj g_sync422_TransObj[SYNC422_PORTNUM];
static Scheduler_arg rdSchePrm;

static void delay1ms(void)
{
	struct timeval timeout;
	
	//delay 1ms
	timeout.tv_sec   = 0;
	timeout.tv_usec  = 1000;	 // 1ms
	select(0, NULL, NULL, NULL, &timeout);
}

static int spi_dev_write(long context, unsigned char *buf, int len)
{
	Sync422_TransObj *pObj = (Sync422_TransObj *)context;
	int SendCnt = 0, TransNum = 0, errCnt = 0, SendTotal = 0;
	int TailLength = 0, numS = 0, waitMs = 0;
	char caCrc = 0;
	Uint32 t1=0, t2=0;

	if(pObj->fd <= 0)
		return -1;

	if(!buf || len <= 0)
	{
		printf(" empty buf canncel sync422 send ! \n");
		return -1;
	}

	Uint8 *p_buf = (Uint8 *)pObj->data_buf;
	if(buf != p_buf)
		memcpy(p_buf, buf, len);
	numS = len;

	// add packet interval
	if(numS % 4)
	{
		TailLength = 4-(numS%4);
		memset(p_buf+numS, 0xFF, TailLength);
		numS += TailLength;
	}
	// add packet interval end

#if SPIDEVON
	t1=OSA_getCurTimeInMsec();
	spi_dataTransform2(pObj->tx_buf, (Uint8 *)(p_buf), numS);
	SendTotal = write(pObj->fd, pObj->tx_buf, numS);
	if(SendTotal != numS)
		errCnt = SendTotal;
	t2=OSA_getCurTimeInMsec();
#endif

#if DEBUGPRI
	{
		#if (DATAFORMAT == DATAFORMAT1)
		ENC_EVENTHEADER *pDataHead = (ENC_EVENTHEADER *)p_buf;
		#elif  (DATAFORMAT == DATAFORMAT2)
		ENC_EVENTHEADER *pDataHead = (ENC_EVENTHEADER *)(p_buf+3);		// ENC_EVENTHEADER start from ENC_USR54HEADER.data
		#endif
		//if(((t2-t1) > 0) || ((pDataHead->transno % 25) == 1))
		if((t2-t1) > 1)
		{
			printf(" spi[%d] dtype[%x] packet[%04x] len:%d spiuse %dms\n", 
					pObj->spiuart, pDataHead->dtype[1], pDataHead->transno, SendTotal, (t2-t1));
		}
	}
#endif
	if(errCnt){
		//printf(" send data failed %d \n", errCnt);
		return -1;
	}

	return (SendTotal-TailLength);
}

static void spi_dev_open(Sync422_TransObj *pObj, const char *dev_name)
{
	pObj->spiTrans.bits_per_word = 32;
	pObj->spiTrans.delay_usecs   = 0;
	//pObj->spiTrans.speed_hz 	 = 40000000;
	pObj->spiTrans.speed_hz = 60*1000000;	// spi_speed = _MHz

	pObj->spiTrans.rx_buf 	=0;// (unsigned long)pObj->rx_buf;
	pObj->spiTrans.tx_buf 		 = (unsigned long)pObj->tx_buf;
	pObj->spiTrans.len			 = 0;
	pObj->spiTrans.cs_change     = 0;

	pObj->fd = spi_init(dev_name, pObj->spiTrans.bits_per_word, pObj->spiTrans.speed_hz);
	if (pObj->fd < 0)
	{
		printf(" [DEBUG:] %s Can't Open SPI Port %s fd[%d]!\n", __func__, dev_name, pObj->fd);
		return ;
	}
	else
	{
		spi_cmd_arg init_user_arg;
		int ret = -1;
		// check fpga version
		init_user_arg.io_addr = FPGA_RD_VESRSION;
		ret = ioctl(pObj->fd, SPI_RD_IOC_SET, &init_user_arg);
		if (ret == -1)
		{
			printf("can't read fpga version !\n");
			spi_close(pObj->fd);
			pObj->fd = 0;
			return ;
		}
	}
	printf(" [DEBUG:] %s Open SPI Port %s fd[%d] success\n", __func__, dev_name, pObj->fd);
}

static void spi_dev_close(Sync422_TransObj *pObj)
{
	if(pObj->fd > 0)
		spi_close(pObj->fd);
	pObj->fd = 0;
}

static void setSync422_fifo_clear(Sync422_TransObj *pObj)
{
	int ret = -1;
	spi_cmd_arg init_user_arg;

	if(pObj->fd <= 0)
		return ;
	// set fpga fifo reset
	init_user_arg.io_addr = FPGA_WR_FIFO_RESET;
	ret = ioctl(pObj->fd, SPI_WR_IOC_SET, &init_user_arg);
	if (ret == -1)
		printf("can't ioctl fpga_fifo clear\n");

	return ;
}

static void setSync422_fifo_limit(Sync422_TransObj *pObj, int upLimit, int downLimit) ////14336  2048
{
	int ret = -1;
	spi_cmd_arg init_user_arg;

	if(pObj->fd <= 0)
		return ;
	// set fpga fifo limit
	init_user_arg.io_addr = FPGA_WR_FIFO_UPPER;
	init_user_arg.io_buffer = upLimit;
	ret = ioctl(pObj->fd, SPI_WR_IOC_SET, &init_user_arg);
	if (ret == -1)
		printf("can't set uplimit\n");
	init_user_arg.io_addr = FPGA_WR_FIFO_FLOOR;
	init_user_arg.io_buffer = downLimit;
	ret = ioctl(pObj->fd, SPI_WR_IOC_SET, &init_user_arg);
	if (ret == -1)
		printf("can't set downlimit\n");

	return ;
}

static void setSync422_clock(Sync422_TransObj *pObj, int iclock)
{
	int ret = -1;
	Uint32 set = 0;
	int ispClock = iclock%0x03;
	spi_cmd_arg init_user_arg;

	pObj->dataClock = iclock;

	if(pObj->fd <= 0)
		return ;
	// set fpga fifo clock
	init_user_arg.io_addr = FPGA_WR_CLOCK;
	init_user_arg.io_buffer = ispClock;
	ret = ioctl(pObj->fd, SPI_WR_IOC_SET, &init_user_arg);
	if (ret == -1)
		printf("can't set clock\n");

	return ;
}

static void* sync422_spi_sendTask(void *pPrm)
{
	Sync422_TransObj *pObj = (Sync422_TransObj *)pPrm;
	//int64 curTime = 0, lastTime = 0;
	int irunCnt=0;

	unsigned char *pIn=NULL;
	int bufId=0, iRtn=0;
	int inLen=0, rtnLen=0;

	int pktLen=0, rdLen=0;
	int splitCnt=0, runCnt=0;

	struct timeval timeout;
	int dtype=0;
	int chTransno[2] = {0, 0};

	Uint8 *p_buf = NULL;
	int HeadLength = 0, numS = 0, i = 0;
	char caCrc = 0;
	int UsrHeadLen = 0, UsrTailLen = 0, numP = 0;
	Uint8 *p_bufPiece = NULL;

	pObj->DataHead.sync[3] = 0x01;
	pObj->DataHead.sync[4] = 0x4E;
	pObj->DataHead.sync[5] = 0x01;
	pObj->DataHead.res[0] = pObj->DataHead.res[1] = 0xAA;
	HeadLength = sizeof(ENC_EVENTHEADER);

	pObj->UsrHead.syncHead[0] = 0xEB;
	pObj->UsrHead.syncHead[1] = 0x90;
	pObj->UsrHead.syncEnd = 0x6F;
	UsrHeadLen = sizeof(ENC_USR54HEADER);

	OSA_printf(" %d:%s spi port %d start. \r\n", OSA_getCurTimeInMsec(), __func__, pObj->spiuart);
	while (pObj->tskLoop == TRUE)
	{
		iRtn = OSA_bufGetFull(&pObj->ringQue, &bufId, TIME_OUT/*OSA_TIMEOUT_FOREVER*/); 
		irunCnt++;
#if DEBUGPRI
		if(irunCnt % 5)
		{
			OSA_printf(" %d:%s run %d clock %d(%d)\r\n", OSA_getCurTimeInMsec(), __func__, irunCnt, pObj->dataClock, iChangeSpeed[pObj->spiuart]);
		}
#endif

#if SPIDEVON
		if(pObj->dataClock != iChangeSpeed[pObj->spiuart])
		{
			if(pObj->spiuart == 0)
				setSync422_clock(pObj, iChangeSpeed[pObj->spiuart]);
		}
#endif

		if(iRtn == OSA_SOK)
		{
			pIn = (unsigned char *)pObj->ringQue.bufInfo[bufId].virtAddr;
			inLen = pObj->ringQue.bufInfo[bufId].size;

			// for PROJ_AXGS040
			//isKeyFrame = pObj->ringQue.bufInfo[bufId].isKeyFrame;
			dtype = pObj->ringQue.bufInfo[bufId].flags;
			//pObj->DataHead.transno = (pObj->DataHead.transno+1)%0xFFFF;
			chTransno[dtype] = (chTransno[dtype]+1)%0xFFFF;
			pObj->DataHead.transno = chTransno[dtype];
			if(dtype == 0)
				pObj->DataHead.dtype[0] = pObj->DataHead.dtype[1] = 0x11;	// TV
			else
				pObj->DataHead.dtype[0] = pObj->DataHead.dtype[1] = 0x22;	// FR

			numS = inLen + HeadLength;
			pObj->DataHead.pktsize[0] = (numS&0xFF);
			pObj->DataHead.pktsize[1] = (numS&0xFF00)>>8;
			pObj->DataHead.pktsize[2] = (numS&0xFF0000)>>16;
			pObj->DataHead.pktsize[3] = (numS&0xFF000000)>>24;
			caCrc = 0;
			for(i = 0; i < inLen; i++)
				caCrc ^= pIn[i];
			pObj->DataHead.res[1] = (caCrc & 0xFF);
			//p_buf = (Uint8 *)pObj->data_buf;
			//memcpy(p_buf, &pObj->DataHead, HeadLength);
			//memcpy(p_buf+HeadLength, pIn, inLen);

			#if (DATAFORMAT == DATAFORMAT1)
			//////////////////////////////////////////
			if(numS >= RING_VIDEO_BUFLEN)
			{
				printf(" over buf canncel sync422-%d send ! \n", pObj->spiuart);
				OSA_bufPutEmpty(&pObj->ringQue, bufId);
				continue;
			}
			p_buf = (Uint8 *)pObj->data_buf;
			memcpy(p_buf, &pObj->DataHead, HeadLength);
			memcpy(p_buf+HeadLength, pIn, inLen);

			#elif  (DATAFORMAT == DATAFORMAT2)
			//////////////////////////////////////////
			numP = numS / 252;	// len of ENC_USR54HEADER.data
			if(numS % 252)
				numP += 1;
			if((numP*UsrHeadLen) >= RING_VIDEO_BUFLEN)
			{
				printf(" over buf canncel sync422-%d send ! \n", pObj->spiuart);
				OSA_bufPutEmpty(&pObj->ringQue, bufId);
				continue;
			}

			p_buf = (Uint8 *)pObj->data_buf;
			p_bufPiece = (Uint8 *)pIn;
			numS = numP*UsrHeadLen;
			for(i = 0; i < numP; i++)
			{
				if(i == 0)
				{
					// first piece
					if(inLen <= 236)		//only one piece
						UsrTailLen = inLen;
					else
						UsrTailLen = 236;	// 252-HeadLength
					memset(pObj->UsrHead.data, 0xFF, 252);
					memcpy(pObj->UsrHead.data, &pObj->DataHead, HeadLength);	// note HeadLength < 252
					memcpy(&pObj->UsrHead.data[HeadLength], p_bufPiece, UsrTailLen);
				}
				else if(i == (numP-1))
				{
					// last piece
					UsrTailLen = (inLen-236) % 252;
					if(UsrTailLen == 0)
						UsrTailLen = 252;
					memset(pObj->UsrHead.data, 0xFF, 252);
					memcpy(pObj->UsrHead.data, p_bufPiece, UsrTailLen);
				}
				else
				{
					// middle piece
					UsrTailLen = 252;
					memset(pObj->UsrHead.data, 0xFF, 252);
					memcpy(pObj->UsrHead.data, p_bufPiece, UsrTailLen);
				}
				pObj->UsrHead.cntno = (pObj->UsrHead.cntno+1)%0xFF;
				p_bufPiece += UsrTailLen;
				//printf(" piece[%d] addr %lx len=%d\n", i, (long)p_bufPiece, UsrTailLen);
				memcpy((p_buf+(i*UsrHeadLen)), &pObj->UsrHead, UsrHeadLen);
			}
#if DEBUGPRI
			if((pObj->DataHead.transno % 250) == 1)
			{
				printf(" send dtype[%x] packet[%04x] len:%d to spidev\n", 
						pObj->DataHead.dtype[1], pObj->DataHead.transno, numS);
			}
#endif
			#endif
			// for PROJ_AXGS040 end

			if(!pObj->dataPause)
				rtnLen = spi_dev_write((long)pObj, pObj->data_buf, numS);
				//rtnLen = Sched_h26x_Scheder(dtype, pObj->data_buf, numS, 0, rdSchePrm.pktPiece, spi_dev_write, (long)pObj);
			if(rtnLen != numS)
				printf(" send dtype[%x] packet[%04x] len:%d to spidev failed %d\n",
						pObj->DataHead.dtype[1], pObj->DataHead.transno, numS, rtnLen);

			OSA_bufPutEmpty(&pObj->ringQue, bufId);
		}
	}

	pObj->tskLoop = FALSE;
	pObj->istskStopDone = TRUE;
	OSA_printf(" %d:%s exit. \r\n", OSA_getCurTimeInMsec(), __func__);
	return 0;
}

int sync422_spi_create(int uart, int mode)
{
	int status, i;
	Sync422_TransObj *pObj = &g_sync422_TransObj[uart];
	memset(pObj, 0, sizeof(Sync422_TransObj));

	memset(&rdSchePrm, 0, sizeof(rdSchePrm));
	rdSchePrm.chMask = 0x01;		// default is onlytv
	rdSchePrm.chfps[0] = rdSchePrm.chfps[1] = 30;   // default is 30 fps
	Sched_h26x_Init();

	pObj->tx_buf = (Uint8 *)malloc(RING_VIDEO_BUFLEN);
	if(!pObj->tx_buf)
		printf("malloc tx mem failed ! size[%d]\n", RING_VIDEO_BUFLEN);
	else
		memset(pObj->tx_buf, 0, RING_VIDEO_BUFLEN);
	pObj->data_buf = (Uint8 *)malloc(RING_VIDEO_BUFLEN);
	if(!pObj->data_buf)
		printf("malloc data mem failed ! size[%d]\n", RING_VIDEO_BUFLEN);
	else
		memset(pObj->data_buf, 0, RING_VIDEO_BUFLEN);

	pObj->spiuart = uart;
#if SPIDEVON
	//if(pObj->spiuart == 0)
	{
		// SPI_PORT_A
		spi_dev_open(pObj, spidevA);
		setSync422_fifo_limit(pObj, SYNC422_FIFO_UPLIMIT_A, SYNC422_FIFO_DNLIMIT_A);//14336  2048
		setSync422_fifo_clear(pObj);
		setSync422_clock(pObj, SYNC422_CLOCK_4M);	// default use 4Mb
		iChangeSpeed[pObj->spiuart] = SYNC422_CLOCK_4M;
		printf(" set sync422-%d speed %d\n", pObj->spiuart, pObj->dataClock);
	}
#endif
	/** < create ringbuf task loop */
	//if(mode == RING_VIDEO)
	{
		pObj->ringCreate.numBuf = 6;
	}
	for (i = 0; i < pObj->ringCreate.numBuf; i++)
	{
		pObj->ringCreate.bufVirtAddr[i] = (void *)malloc(RING_VIDEO_BUFLEN);
		OSA_assert(pObj->ringCreate.bufVirtAddr[i] != NULL);
	}
	if(OSA_bufCreate(&pObj->ringQue, &pObj->ringCreate) != OSA_SOK)
		printf("ringbuf create failed ! \n");
	pObj->tskLoop = TRUE;
	pObj->istskStopDone = FALSE;
	status = OSA_thrCreate(
				 &pObj->tskHndl,
				 sync422_spi_sendTask,
				 0,
				 0,
				 pObj
			 );
	OSA_assert(status == OSA_SOK);

	// ibInit = 1;
	ibInit |= (1<<uart);
	return status;
}

int sync422_spi_destory(int uart)
{
	int status, i;
	Sync422_TransObj *pObj = &g_sync422_TransObj[uart];

	ibInit &= ~(1<<uart);

	status = OSA_thrDelete(&pObj->tskHndl);
	OSA_assert(status == OSA_SOK);
#if SPIDEVON
	spi_dev_close(pObj);
#endif
	free(pObj->tx_buf);
	//free(pObj->rx_buf);
	free(pObj->data_buf);

	OSA_bufDelete(&pObj->ringQue);
	for(i=0; i<pObj->ringCreate.numBuf; i++)
	{
		if(pObj->ringCreate.bufVirtAddr[i] != NULL)
		{
			free(pObj->ringCreate.bufVirtAddr[i]);
			pObj->ringCreate.bufVirtAddr[i] = NULL;
		}
	}

	return status;
}

int sync422_spi_speed(int uart, int ispeed)
{
	if((ibInit & (1<<uart)) == 0)
		return -1;
	if(ispeed< SYNC422_CLOCK_2M || ispeed > SYNC422_CLOCK_8M)
		return -1;

	int clockVal = (1<<(ispeed+1));
	iChangeSpeed[uart] = ispeed;
	OSA_printf(" %d:%s set sync422-%d speed %dM\n", OSA_getCurTimeInMsec(), __func__, uart, clockVal);
	return 0;
}

int sync422_spi_pause(int uart, int ipause)
{
	if((ibInit & (1<<uart)) == 0)
		return -1;
	
	Sync422_TransObj *pObj = &g_sync422_TransObj[uart];
	pObj->dataPause = (ipause != 0)?1:0;
	printf(" %d:%s set sync422-%d pause %d\n", OSA_getCurTimeInMsec(), __func__, uart, ipause);
	return 0;
}

int sync422_ontime_video(int dtype, unsigned char *buf, int len)
{
#if DEBUGPRI
	printf("[sync422]%s: dtype=0x%x buf=%p len=%d(%d); ibInit=0x%x\n",
			__func__, dtype, buf, len, ENC_VIDEO_BUFMAX, ibInit);
#endif
	if((ibInit & 0x01) == 0)
		return -1;
	if(buf == NULL)
		return -1;
	if(len >= ENC_VIDEO_BUFMAX)
	{
		printf("copy video dtype[%x] data len%d overflow \n", dtype, len);
		return -1;
	}

	Sync422_TransObj *pObj = &g_sync422_TransObj[RING_VIDEO_TV];
	int status, i;
	int bufId=0, iRtn=0;
	unsigned char *pOut=NULL;
	iRtn = OSA_bufGetEmpty(&pObj->ringQue, &bufId, OSA_TIMEOUT_NONE/*OSA_TIMEOUT_FOREVER*/);
	if(iRtn == OSA_SOK)
	{
		pOut = (unsigned char *)pObj->ringQue.bufInfo[bufId].virtAddr;
		memcpy(pOut, buf, len);
		pObj->ringQue.bufInfo[bufId].size = len;
		pObj->ringQue.bufInfo[bufId].flags = dtype;	// TV-0 or FR-1
		OSA_bufPutFull(&pObj->ringQue, bufId);
		return len;
	}
	else
	{
		OSA_printf(" [video] ringbuf no space lost packet!!\n");
		return 0;
	}
}

int sync422_ontime_sche(int uart)
{
	int iuartrate = 0, ichfps[2] = {0, 0};

	// formula (uartbitrate) / (n)fps
	if (iChangeSpeed[uart] == SYNC422_CLOCK_4M)
		iuartrate = 480000;
		//iuartrate = (3.84*1000*1000/8);
	else if (iChangeSpeed[uart] == SYNC422_CLOCK_8M)
		iuartrate = 960000;
		//iuartrate = (7.68*1000*1000/8);
	else /*if (iChangeSpeed[uart] == SYNC422_CLOCK_2M)*/
		iuartrate = 240000;
		//iuartrate = (1.92*1000*1000/8);

	if (rdSchePrm.chMask & 0x1)
		ichfps[0] = rdSchePrm.chfps[0];
	if (rdSchePrm.chMask & 0x2)
		ichfps[1] = rdSchePrm.chfps[1];
	rdSchePrm.pktPiece = iuartrate / (ichfps[0] + ichfps[1]);

	if ((rdSchePrm.chMask & 0x03) == 0x03)
	{
		Sched_config(0, 2, ichfps[0]);
		Sched_config(1, 2, ichfps[1]);
	}
	else
	{
		Sched_config(0, 1, ichfps[0]);
		Sched_config(1, 1, ichfps[1]);
	}
	return 0;
}

int sync422_ontime_ctrl(CTRL_T icmd, int dtype, int iprm)
{
	int uart=0;

	if((ibInit & (1<<uart)) == 0)
		return -1;

	if(icmd == ctrl_prm_uartrate)
	{
		if(iprm >= SYNC422_CLOCK_2M && iprm <= SYNC422_CLOCK_8M)
		{
			iChangeSpeed[uart] = iprm;
			OSA_printf(" %d:%s set sync422-%d speed %dM\n", OSA_getCurTimeInMsec(), __func__, uart, (2<<iChangeSpeed[uart]));
		}
	}

	if(icmd == ctrl_prm_framerate)
	{
		if(iprm >= 15 && iprm <= 30)
		{
			rdSchePrm.chfps[dtype%2] = iprm;	// TV-0 or FR-1
			OSA_printf(" %d:%s set sync422-%d chl %d fps %d\n", OSA_getCurTimeInMsec(), __func__, uart, dtype, rdSchePrm.chfps[dtype%2]);
		}
	}

	if(icmd == ctrl_prm_chlMask)
	{
		if(iprm != 0)
		{
			rdSchePrm.chMask = iprm&0x03;
			OSA_printf(" %d:%s set sync422-%d chlMask %x\n", OSA_getCurTimeInMsec(), __func__, uart, rdSchePrm.chMask);
		}
	}

	sync422_ontime_sche(uart);
	return 0;
}

#if DEBUGMODE
static OSA_ThrHndl demoTskHndl[2];
static bool demoTskLoop[2];
static bool demoTskStopDone[2];
static int sndDbg[2] = {0, 0};
static int demoIntervalDelay = 20;

static void* Sync422_sendTask_demo(void *pPrm)
{
	//struct timeval timeout;
	//int64 curTime = 0, lastTime = 0;

	//Sync422_TransObj *pObj = &g_sync422_TransObj[0];
	int dtype=*(int *)pPrm;
	FILE *fp_video;
	unsigned long filesize=-1;
	unsigned char *rdBuf=NULL;
	int pktLen=64*1024; // set Bytes per packet
	int rdLen=0, rtnLen=0, totalLen=0;
	int splitCnt=0, runCnt=0;
	UInt8 *pStart = NULL,*pEnd = NULL;
	int tmpSdLen=0;
	Uint32 t3=0, t4=0;

	rdBuf = (unsigned char *)malloc(0xC00000);	// 12MB
	OSA_assert(rdBuf != NULL);

	if(dtype == RING_VIDEO_TV)
		fp_video = fopen("1.h265","rb");
	else
		fp_video = fopen("2.h265","rb");
	if(fp_video != NULL)
	{
		fseek(fp_video,0L,SEEK_END);
		filesize = ftell(fp_video);
		fseek(fp_video,0L,SEEK_SET);
		rdLen = fread(rdBuf,1,0xC00000,fp_video);
	}
	else
	{
		filesize = 0;
		printf("open data file failed\n");
	}

	OSA_printf(" %d:%s dtype %d start!!!\r\n", OSA_getCurTimeInMsec(), __func__, dtype);
	while (demoTskLoop[dtype] == TRUE)
	{
		if(sndDbg[dtype])
		{
			if(filesize == 0)
			{
				sndDbg[dtype] = 0;
				OSA_waitMsecs(1000); //1000ms
				continue;
			}
#if 0
			/*********************/
			// divide data and send
			splitCnt = (filesize/pktLen);
			if(filesize%pktLen)
			{
				splitCnt++;	// last packet
			}
			rdLen = filesize - (splitCnt-1)*pktLen;
			
			runCnt = 0;
			totalLen = 0;
			//printf(" test send %d need split %d\n", (filesize+splitCnt*16), splitCnt);
			while(splitCnt > 0)
			{
				if(splitCnt == 1)	// last packet
				{
					//rtnLen = spi_dev_write_withdelay((long)pObj, rdBuf+(runCnt*pktLen), rdLen);
					rtnLen = sync422_ontime_video(dtype, rdBuf+(runCnt*pktLen), rdLen);
				}
				else
				{
					//rtnLen = spi_dev_write_withdelay((long)pObj, rdBuf+(runCnt*pktLen), pktLen);
					rtnLen = sync422_ontime_video(dtype, rdBuf+(runCnt*pktLen), pktLen);
				}

				if(rtnLen >= 0)
				{
					totalLen += rtnLen;
				}
				splitCnt--;
				runCnt++;
			}
			printf(" test send %d end split %d\n", totalLen, runCnt);
			// divide data and send end
			/*********************/
#else
			/*********************/
			// frame data and send
			pStart   = rdBuf;
			pEnd     = rdBuf+1;
			runCnt = 0;
			totalLen = 0;
			tmpSdLen = 0;
			t3 = OSA_getCurTimeInMsec();
			while(demoTskLoop[dtype] == TRUE && (sndDbg[dtype]))
			{
				if(pEnd[0] == 0x00u && pEnd[1] == 0x00u && pEnd[2] == 0x00u && pEnd[3] == 0x01u)
				{
					pktLen = pEnd - pStart;
					//printf(" databuf cnt=%d pkt len=%d\n", runCnt, pktLen);
					//rtnLen = spi_dev_write_withdelay((long)pObj, pStart, pktLen);
					rtnLen = sync422_ontime_video(dtype, pStart, pktLen);
					if(demoIntervalDelay > 0)
						OSA_waitMsecs(demoIntervalDelay);
					if(rtnLen >= 0)
					{
						totalLen += rtnLen;
					}
					runCnt++;
					pStart = pEnd;
				}
				else
				{
					if((rdBuf + filesize - pEnd) < 4)
					{
						// last packet
						rdLen = rdBuf + filesize - pStart;
						//printf(" databuf last cnt=%d pkt len=%d\n", runCnt, rdLen);
						//rtnLen = spi_dev_write_withdelay((long)pObj, pStart, rdLen);
						rtnLen = sync422_ontime_video(dtype, pStart, rdLen);
						if(demoIntervalDelay > 0)
							OSA_waitMsecs(demoIntervalDelay);
						if(rtnLen >= 0)
						{
							totalLen += rtnLen;
						}
						runCnt++;
						break;
					}
				}
				pEnd++;
			}
			printf(" test send %d end split %d\n", totalLen, runCnt);
			// frame data and send end
			/*********************/
#endif
			if(sndDbg[dtype] == 1)
				sndDbg[dtype] = 0;
		}
		OSA_waitMsecs(100);
	}

	if(rdBuf != NULL)
		free(rdBuf);

	if(fp_video != NULL)
	{
		fclose(fp_video);
		printf(" close fp_video\n");
	}

	demoTskStopDone[dtype] = TRUE;
	return 0;
}

int sync422_demo_start(void)
{
	int status, i;

	sync422_ontime_ctrl(ctrl_prm_uartrate, 0, SYNC422_CLOCK_8M);

	for(i=0; i<2; i++)
	{
		sndDbg[i] = 0;
		demoTskLoop[i] = TRUE;
		demoTskStopDone[i] = FALSE;
		status = OSA_thrCreate(
					 &demoTskHndl[i],
					 Sync422_sendTask_demo,
					 0,
					 0,
					 &i
				 );
		OSA_assert(status == OSA_SOK);
		OSA_waitMsecs(100);
	}
	return 0;
}

int sync422_demo_stop(void)
{
	int i;

	for(i=0; i<2; i++)
	{
		if(demoTskLoop[i] == TRUE)
		{
			demoTskLoop[i] = FALSE;
			while(!demoTskStopDone[i])
			{
				OSA_waitMsecs(40);
			}
			OSA_thrDelete(&demoTskHndl[i]);
		}
	}
	return 0;
}

void testSnd(int ichl, int mode)
{
	sndDbg[ichl] = mode;

	{
		int chlMask = 0;
		if(sndDbg[0])
			chlMask |= 0x1;
		if(sndDbg[1])
			chlMask |= 0x2;
		sync422_ontime_ctrl(ctrl_prm_chlMask, 0, chlMask);
	}
}

void testSpeed(int ispeed)
{
	sync422_ontime_ctrl(ctrl_prm_uartrate, 0, ispeed);
}

#endif

