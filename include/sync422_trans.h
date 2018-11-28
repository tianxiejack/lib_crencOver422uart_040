

#ifndef _SYNC422_TRANS_H_
#define _SYNC422_TRANS_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/types.h>

typedef enum
{
    ctrl_prm_uartrate = 0x00,	// iPrm: 0-2M 1-4M 2-8M
    ctrl_prm_framerate = 0x01,	// dtype: 0-tv 1-fr  // iPrm: fps-15/25/30
    ctrl_prm_chlMask = 0x02,	// iPrm: bit0-tv bit1-fr
}CTRL_T;

int sync422_spi_create(int uart, int mode);	// uart: 0-A 1-B  // mode: 0-vid 1-pho
int sync422_spi_destory(int uart);
int sync422_ontime_video(int dtype, unsigned char *buf, int len);
int sync422_ontime_ctrl(CTRL_T icmd, int dtype, int iprm);

///////////
int sync422_demo_start(void);
int sync422_demo_stop(void);
void testSnd(int ichl, int mode);
void testSpeed(int ispeed);


#endif

