#ifndef __SPD_COMM__
#define __SPD_COMM__

#include "stm32f4xx.h"

#define SPD1_QUEUE_LEN 10

//速度計算值隊列
typedef struct _tag_speed_queue
{
    u16 ptr_head;
    u16 ptr_tail;
    long lSum_ang;
    long lSum_tim;
    short queue_ang[SPD1_QUEUE_LEN];
    short queue_tim[SPD1_QUEUE_LEN];
} SpeedValueQueue;

//pid调节器结构
typedef struct _tag_PID_Module
{

    short paraP; //参数：比例
    short paraI; //参数：积分
    short paraD; //参数：微分

    short paraSet;    //参数：设定值
    short paraUpLmt;  //参数：输出上限值
    short paraDnLmt;  //参数：输出下限值
    short paraResver; //参数：作用方式
    short paraAuto;   //参数：调节器工作
    short paraManVal; //参数：手动值

    short valIn;  //参数输入值
    short valOut; //参数输出值

    int vOutL1;     //上一次计算值
    int vOutL2;     //上二次计算值
    short sDeltaL1; //上一次偏差值
    short sDeltaL2; //上二次偏差值
} PID_Module;

//-----------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq);
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim);
short SpdQueueAvgVal(SpeedValueQueue *svq);

//--------------------------------------------------------------------
void PIDMod_initialize(PID_Module *pPid, int no);  //PID模块初始化
void PIDMod_step(PID_Module *pPid);                //PID模块计算
void PIDMod_update_para(PID_Module *pPid, int no); //PID模块参数更新

#endif
/*------------------end of file------------------------*/
