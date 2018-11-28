/**************************************************************************************************
* Copyright (C), 2011-2012, ChamRun Tech. Co., Ltd.
* WorkContext:
* FileName:    snd_scheduler.c
* Author:      wzk       Version :          Date:2015年9月7日
* Description: //  对2M/4M/8M H.264的码流做发送调度。
*              //  1. H.264码流在编码的时候有一个周期为11帧的16ms抖动
*              //  2. 422 2M 串口一个周期33ms 只能发送8.34K,需要让带宽利用率更高,
*              //     通过调度可以使数据发送更均匀
*              //  调度策略
*                  1. 根据发送数据长度计算传输时间,每一短数据帧发送延迟时间t
*                     t = (两帧之间能发送字节数 - 当前等待发送字节) * 帧间隔时间 / 两帧之间能发送字节数
*                     以30fps为例
*                           t = (8.34k - frameLen) * 33ms / 8.34k
*                  2. 视频数据每次到达不直接启动数据发送,只负责放入发送队列
*                     起一个以视频周期为单位的定时器,当定时器到达,查看缓存数据,启动发送,可去除编码抖动
*
*              //注意:策略2 增加了一帧的延迟, 遇到大数据对应用程序无影响,会在驱动中积攒
*                    :不支持多个通道
* Version:     //  版本信息
* History:     //  历史修改记录
* <author> 		<time> 			<version > 		<description>
* wzk  		2015年9月7日          V1.00    		build this module
***************************************************************************************************/
#include <unistd.h>
#include <assert.h>
#include <osa.h>
#include <osa_tsk.h>
#include <vidScheduler.h>
#include <math.h>

#define MAX_FRAMES      9
#define FRAME_BUFLEN	0x040000	// 256KB

typedef struct
{
    unsigned int tm_input;
    unsigned int tm_output;
    unsigned int tm_schedBegin;
    unsigned int tm_schedEnd;
    unsigned int tm_cap;
    unsigned int tm_wait;   //422发送调度等待时间
    unsigned char *data;    //数据指针
    unsigned int lengh;     //数据长度
}Sched_Frame;

typedef struct
{
    int index;
    int rdIndx;
    int wrIndx;
    int framewait;
    OSA_SemHndl sem;
    OSA_MutexHndl muxLock;
    OSA_ThrHndl thread;
    int taskLoop;

    int thrd;
    Sched_Frame frames[MAX_FRAMES];

    int sendTime_isBuild;
    unsigned int sendTime_sndBase;
    unsigned int sendTime_capBase;

//    FILE *cbFd;
//    size_t (*write_cb)(const void *, size_t, size_t, FILE* );
    long cbFd;
    int (*write_cb)(long fd, unsigned char *buf, int len);
}
Sched_Chn;

//打开DM8168编码抖动去除,使能定时器驱动发送,不使用视频码流驱动发送
//#define SCHEDER_H264_ENC

#define TIME_DIFF(t0,t1) \
        (( t1 - t0) > 0 )?(t1 - t0) : 0

//extern void memcpy_neon( void *, const void *, size_t );

#define CHAN_COUNT 2
static Sched_Chn schedChannel[CHAN_COUNT];
static int bInit = 0;

//static int last_snd_ts[2]={0,0};
//static int ch_snd_td[2]={33,33};
//static unsigned int g_cap_tm[2] = {0,0};

static OSA_ThrHndl thread_tm;
static int taskLoop_tm = 0;

static int g_chan_cout = 2;

static void* SchedTask(void *pPrm);
static void * SchedTimeTask(void *pPrm);
static unsigned int Sched_get_wait_time(Sched_Chn * sched, unsigned int capTs,unsigned int delayTs, unsigned int *pBegin, unsigned int *pEnd);

/*
* @Function:    //  Sched_h265_Init
* @Description: //  分配视频帧队列,启动调度发送任务
* @Return:      //  成功失败
* @Others:      //  队列为8帧,cache 帧为1帧
* @Others:      //  外部接口函数
*/
int Sched_h26x_Init()
{
    int i = 0, j=0;
    int status;
    Sched_Chn *pCh=NULL;

    if(bInit)
        return 0;

    /* 分配缓存*/
    memset(schedChannel, 0, sizeof(schedChannel));

    for( j = 0;j < CHAN_COUNT; j++)
    {
        pCh = &schedChannel[j];
        pCh->index = j;			
        for( i = 0; i < MAX_FRAMES; i++)
        {
            pCh->frames[i].data =(unsigned char*)malloc(FRAME_BUFLEN);
            OSA_assert(pCh->frames[i].data!=NULL);

            pCh->frames[i].lengh = 0;
        }

        pCh->framewait = 1000000/30;	// default is 30fps

        /* 创建同步信号量*/
        status = OSA_semCreate(&pCh->sem , 1, 0);
        OSA_assert(status == OSA_SOK);

        /* 创建互斥锁*/
        status = OSA_mutexCreate(&pCh->muxLock);
        OSA_assert(status == OSA_SOK);

        /* 创建调度发送任务*/
        status = OSA_thrCreate(
                     &pCh->thread,
                     SchedTask,
                     OSA_THR_PRI_DEFAULT,
                     0,
                     pCh
                 );
        OSA_assert(status==OSA_SOK);
    }

    /* 创建调度发送任务*/
	taskLoop_tm  = 0;
    status = OSA_thrCreate(
                 &thread_tm,
                 SchedTimeTask,
                 OSA_THR_PRI_DEFAULT,
                 0,
                 NULL
             );
    OSA_assert(status==OSA_SOK);

    bInit = 1;

    return OSA_SOK;
}
/*
* @Function:    //  Sched_h265_unInit
* @Description: //  释放视频帧队列,注销调度发送任务
* @Others:      //  外部接口函数
*/
int Sched_h26x_unInit()
{
    int i = 0, j=0;

    Sched_Chn *pCh=NULL;

    if(!bInit)
        return OSA_SOK;
	
    taskLoop_tm  = 1;

    OSA_waitMsecs(100);

    OSA_thrDelete(&thread_tm);

    for( j = 0;j < CHAN_COUNT; j++)
    {
        pCh = &schedChannel[j];

        pCh->taskLoop = 1;

        OSA_semSignal(&pCh->sem);

        OSA_waitMsecs(10);

        OSA_thrDelete(&pCh->thread);

        for(i = 0; i < MAX_FRAMES; i++)
        {
            free(pCh->frames[i].data);
        }

        OSA_mutexDelete(&pCh->muxLock);

        OSA_semDelete(&pCh->sem);
    }

    bInit = 0;

    return OSA_SOK;
}
/*
* @Function:    //  Sched_frame_output
* @Description: //  从缓存队列取出一帧,往外发送
* @Input:       //  Sched_Chn* sched 调度器句柄
* @Output:      //  对输出参数的说明。
* @Return:      //  发送数据长度
* @Others:      //  内部函数
*/
static int Sched_frame_output(Sched_Chn* sched, int flag)
{
    int iRet = 0;
    int rd;
    struct timeval timeout;
    timeout.tv_sec   = 0;
    //int delay_tm = 17, currFrames;

    OSA_mutexLock(&sched->muxLock);

    rd = sched->rdIndx ;

    /*有缓存数据启动发送*/
    if(sched->wrIndx != sched->rdIndx )
    {
        int thrd = sched->thrd;
        int tmp = 0;
        int len = sched->frames[rd].lengh;

        /* 计算422串口发送调度时间*/
        if(len < thrd)
        {
            // formula_aloysa 1000*1000/30
            tmp = (thrd - len) * sched->framewait / g_chan_cout / thrd;
            tmp = (tmp > 0) ?tmp : 0;

            sched->frames[rd].tm_wait = tmp;
        }
        else
        {
            //printf("len:%d Bytes, sync422:%d per %dms, send immediately\n", len , thrd, 1000/sched->framewait);
        }

/*
        OSA_printf("chid %d cur tm %05d wait %05d len %05d",
                   sched->index, OSA_getCurTimeInMsec(),sched->frames[rd].tm_wait,len);
*/

        /*422发送调度延迟*/
        if(sched->frames[rd].tm_wait!=0)
        {
            //OSA_waitMsecs(sched->frames[rd].tm_wait);
            OSA_mutexUnlock(&sched->muxLock);
            if (flag)
            {
                timeout.tv_usec = sched->frames[rd].tm_wait;
                select(0,NULL,NULL,NULL,&timeout);
            }
            OSA_mutexLock(&sched->muxLock);
        }

        if(sched->write_cb!=NULL && sched->wrIndx != sched->rdIndx)
        {
            iRet = sched->write_cb(sched->cbFd,sched->frames[rd].data,sched->frames[rd].lengh);
        }

        rd = (rd+ 1)%MAX_FRAMES;
        sched->rdIndx = rd;
    }

    OSA_mutexUnlock(&sched->muxLock);

    return iRet;
}
/*
* @Function:    //  Sched_frame_input
* @Description: //  外部向调度器缓存队列存入一帧,立即返回
* @Input:       //  Sched_Chn* sched 调度器句柄
*               //  void * data 数据指针
*               //  int len 需要写入的数据长度
* @Output:      //  对输出参数的说明。
* @Return:      //  写入数据长度
* @Assumptions: //  不会有任何一帧数据超过256K
* @Others:      //  内部函数
*/
static int Sched_frame_input(Sched_Chn* sched,void * data,int len,unsigned int ts)
{
    int i  = 0;
    int wr =  sched->wrIndx;

    memcpy(sched->frames[wr].data, data, len);
    sched->frames[wr].lengh = len;
    sched->frames[wr].tm_input = OSA_getCurTimeInMsec();

    sched->frames[wr].tm_cap  = ts;

    sched->frames[wr].tm_wait = 0;

    OSA_mutexLock(&sched->muxLock);

    wr = (wr + 1)%MAX_FRAMES;
    sched->wrIndx = wr;

    /* 调度器缓存满直接发送 */
    if(sched->wrIndx == sched->rdIndx)
    {
        OSA_mutexUnlock(&sched->muxLock);
        for(i = 0; i < MAX_FRAMES-1; i++)
        {
            Sched_frame_output(sched, 0);
        }
        OSA_printf("chId %d Sched_frame_input overflow!!!",sched->index);
    }
    else
    {
        OSA_mutexUnlock(&sched->muxLock);
    }

#ifndef SCHEDER_H264_ENC
    //OSA_semSignal(&sched->sem);
#endif

    return len;
}

static void * SchedTimeTask(void *pPrm)
{
    int pp = 0;
    struct timeval timeout;
    timeout.tv_sec   = 0;

    while(!taskLoop_tm)
    {
        if(taskLoop_tm)
            break;

        //timeout.tv_usec = (schedChannel[0].framewait-100) / g_chan_cout;
        timeout.tv_usec = (33333-100) / 2;
        select(0,NULL,NULL,NULL,&timeout);

        OSA_semSignal(&schedChannel[pp].sem);

//        pp^=1;
        pp = (pp + 1)%CHAN_COUNT;
    }
    return NULL;
}
/*
* @Function:    //  SchedTask
* @Description: //  调度器发送任务
* @Input:       //  void *pPrm 调度器句柄
* @Output:      //  无
* @Return:      //  无
* @Others:      //  内部函数
*/
static void* SchedTask(void *pPrm)
{
    int iRet = 0;

    unsigned int tmp = 0;
    int rd = 0;
    int last_ts = 0;
    int curr_ts = 0;
    int isDelay  = 0;

    Sched_Chn * sched = (Sched_Chn*)pPrm;
    struct timeval timeout;
    timeout.tv_sec   = 0;

    printf(" [DEBUG:] %s task entry !!! \r\n",__func__);
    last_ts = OSA_getCurTimeInMsec();

    while(!sched->taskLoop)
    {
        tmp = OSA_getCurTimeInMsec();
        tmp = (tmp >= last_ts) ? (tmp-last_ts) : (last_ts);

#ifndef SCHEDER_H264_ENC
        OSA_semWait(&sched->sem, OSA_TIMEOUT_FOREVER);
#else

        if(isDelay)
        {
            tmp = 5000;//5ms
            isDelay = 0;
        }
        timeout.tv_usec = tmp;
        select(0,NULL,NULL,NULL,&timeout);
#endif

        last_ts = OSA_getCurTimeInMsec();
        //OSA_printf("tm %d", last_ts - curr_ts);
        curr_ts = last_ts;

        if(sched->taskLoop )
            break;

        if(sched->wrIndx!= sched->rdIndx )
        {
            tmp = 0;
            rd = sched->rdIndx;
            iRet = Sched_frame_output(sched, 1);
        }
        else
        {
            isDelay = 0x01;
            //OSA_printf("chId %d Que is empty!!!",sched->index);
        }
    }

    printf(" [DEBUG:] %s task exit!!! \r\n",__func__);
    return NULL;
}
/*
* @Function:    //  Sched_h265_Scheder
* @Description: //  外部调用此函数向调度器缓存队列存入一帧,立即返回
* @Input:       //  iChn 编码通道 未使用
*               //  void * data 数据指针
*               //  int len 需要写入的数据长度
*               //  ts 时间戳未使用
*               //  int thrd 帧周期最大发送字节数
*               //  write_cb 发送回调函数
*               //  cbFd 发送回调函数串口FD
* @Output:      //  对输出参数的说明。
* @Return:      //  写入数据长度
* @Assumptions: //  不会有任何一帧数据超过80K
* @Others:      //  外部接口函数
*/
int Sched_h26x_Scheder
(
    int iChn,
    void *data,
    int len,
    unsigned int ts,
    int thrd,
    int (*write_cb)(long, unsigned char *buf, int),
    long  cbFd
)
{
    int iRet = 0;
    Sched_Chn * sched=NULL;

    if(!bInit)
    {
        Sched_h26x_Init();
    }
    if(len >= FRAME_BUFLEN)
		return iRet;

    OSA_assert(iChn < CHAN_COUNT);

    sched = &schedChannel[iChn];
    //sched = &schedChannel[0];

    sched->write_cb = write_cb;
    sched->cbFd = cbFd;
    sched->thrd = thrd;
    iRet = Sched_frame_input(sched,data,len,ts);

    return iRet;
}

int Sched_config
(
    int iChn,
    int iChn_cout,
    int framerate
)
{
    Sched_Chn * sched=NULL;
    if(!bInit)
        return 0;

    sched = &schedChannel[iChn];

    if(framerate != 0)
    {
        sched->framewait = 1000000 / framerate;
    }

    g_chan_cout = iChn_cout;	// = 1;

    return 0;
}

static unsigned int Sched_get_wait_time(Sched_Chn * sched, unsigned int capTs,unsigned int delayTs, unsigned int *pBegin, unsigned int *pEnd)
{
    unsigned int curTm = 0;
    unsigned int capD = 0;
    unsigned int uRet = 0;

    OSA_assert(sched != NULL);

    curTm = OSA_getCurTimeInMsec();

    if( (!sched->sendTime_isBuild ) || (capTs < sched->sendTime_capBase) ||(curTm < sched->sendTime_sndBase))
    {
        sched->sendTime_sndBase = curTm ;
        sched->sendTime_capBase = capTs;
        sched->sendTime_isBuild = 1;
    }

    capD = capTs - sched->sendTime_capBase;
    uRet =  sched->sendTime_sndBase + capD + delayTs;

    /*    if(uRet <= curTm)
        {
            sched->sendTime_sndBase = curTm ;
            sched->sendTime_capBase = capTs;
            uRet =  sched->sendTime_sndBase + delayTs;
        }*/

    if(pBegin != NULL)
        *pBegin = curTm;
    if(pEnd != NULL)
        *pEnd = uRet;

    return (uRet < curTm) ?  0 :(uRet - curTm)*1000;//us
}
/**********************************************end file****************************************/
