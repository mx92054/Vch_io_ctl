#include "spd_comm.h"

extern short wReg[];

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq)
{
    int i;

    svq->ptr_head = 0;
    svq->ptr_tail = 0;
    svq->lSum_ang = 0;
    svq->lSum_tim = 0;
    for (i = 0; i < SPD1_QUEUE_LEN; i++)
    {
        svq->queue_ang[i] = 0;
        svq->queue_tim[i] = 0;
    }
}

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//          val:插入隊列的值
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim)
{
    svq->lSum_ang += ang;
    svq->lSum_ang -= svq->queue_ang[svq->ptr_head];
    svq->lSum_tim += tim;
    svq->lSum_tim -= svq->queue_tim[svq->ptr_head];

    svq->queue_ang[svq->ptr_head] = ang;
    svq->queue_tim[svq->ptr_head] = tim;

    svq->ptr_head++;
    if (svq->ptr_head >= SPD1_QUEUE_LEN)
        svq->ptr_head = 0;
}

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	隊列中保存數據的平均值
//-------------------------------------------------------------------------------
short SpdQueueAvgVal(SpeedValueQueue *svq)
{
    if (svq->lSum_tim == 0)
        return 0;
    return svq->lSum_ang * 1000 / svq->lSum_tim;
}

/****************************************************************
 *	@brief	PID模块初始化
 *	@param	pPid模块指针
 *          no 参数起始地址
 *	@retval	None
 ****************************************************************/
void PIDMod_initialize(PID_Module *pPid, int no)
{
    pPid->paraP = wReg[no];
    pPid->paraI = wReg[no + 1];
    pPid->paraD = wReg[no + 2];
    pPid->paraSet = wReg[no + 3];
    pPid->paraUpLmt = wReg[no + 4];
    pPid->paraDnLmt = wReg[no + 5];
    pPid->paraResver = wReg[no + 6];
    pPid->paraAuto = wReg[no + 7];
    pPid->paraManVal = wReg[no + 8];

    pPid->vOutL1 = 0;
    pPid->vOutL2 = 0;
    pPid->sDeltaL1 = 0;
    pPid->sDeltaL2 = 0;
}

void PIDMod_step(PID_Module *pPid)
{
    int pid_u, pid_out, lmt;
    short curDelta;

    if (!pPid->paraAuto)
    {
        pPid->valOut = pPid->paraManVal ;
        return;
    }

    curDelta = pPid->valIn - pPid->paraSet; //当前偏差值
    pid_u = pPid->paraP * (curDelta - pPid->sDeltaL1 +
                           pPid->paraI * pPid->sDeltaL1 +
                           pPid->paraD * (curDelta - 2 * pPid->sDeltaL1 + pPid->sDeltaL2));
    pPid->sDeltaL2 = pPid->sDeltaL1;
    pPid->sDeltaL1 = curDelta;

    pid_out = pPid->vOutL1;
    if (pPid->paraResver) //根据作用方式确定是增量还是减量
    {
        pid_out += pid_u;
    }
    else
    {
        pid_out -= pid_u;
    }

    //输出值限幅，避免调节器饱和
    if (pPid->paraUpLmt != 0)
    {
        lmt = pPid->paraUpLmt * 1000;
        if (pid_out > lmt)
            pid_out = lmt;
        if (pid_out < -lmt)
            pid_out = -lmt;
    }

    pPid->valOut = pid_out / 1000;
    pPid->vOutL2 = pPid->vOutL1;
    pPid->vOutL1 = pid_out;
}

void PIDMod_update_para(PID_Module *pPid, int no)
{
    pPid->paraP = wReg[no];
    pPid->paraI = wReg[no + 1];
    pPid->paraD = wReg[no + 2];
    pPid->paraSet = wReg[no + 3];
    pPid->paraUpLmt = wReg[no + 4];
    pPid->paraDnLmt = wReg[no + 5];
    pPid->paraResver = wReg[no + 6];
    pPid->paraAuto = wReg[no + 7];
    pPid->paraManVal = wReg[no + 8];
}

/*------------------end of file------------------------*/
