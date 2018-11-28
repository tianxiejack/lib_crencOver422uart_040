/*******************************************************************************
* Copyright (C), 2011-2012, ChamRun Tech. Co., Ltd.
* WorkContext:
* FileName:    snd_scheduler.h
* Author:      xavier       Version :          Date:2015年9月7日
* Description: //  模块描述
* Version:     //  版本信息
* History:     //  历史修改记录
* <author> 		<time> 			<version > 		<description>
* xavier  		2015年9月7日          V1.00    		build this module
* aloysa		20180828						modify
********************************************************************************/

#ifndef _VID_SCHEDULER_H_
#define _VID_SCHEDULER_H_

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <osa_tsk.h>
#include <osa_sem.h>
#include <osa_mutex.h>
#include <osa_thr.h>

int Sched_h26x_Init();
int Sched_h26x_unInit();
int Sched_h26x_Scheder(int iChn, void *data, int len, unsigned int ts, int thrd,int (*write_cb)(long, unsigned char *buf, int), long cbFd);
int Sched_config(int iChn,int iChn_cout,int framerate);

#endif

/*******************************end file****************************************/

