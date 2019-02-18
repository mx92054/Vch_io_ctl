/*********************************************************
 * �ļ���  ��SysTick.c
 * ����    ��SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
********************************************************/
#include "SysTick.h"
#include "Modbus_svr.h"

#define TIMER_NUM 10

extern short wReg[];

static __IO u32 TimingDelay = 0; // ��ʱ��ʱ��������
__IO u16 TimePre[TIMER_NUM];	 //	������Ԥ��ֵ
__IO u16 TimeCur[TIMER_NUM];	 //	������Ԥ��ֵ
__IO u8 TimerFlag[TIMER_NUM];	// ��������־λ
__IO u32 nlTicks = 0;			 //��ǰ����ʱ��ļ�����

/*---------SysTick Initialize---------------------------------------------------*/
void SysTick_Init(void)
{
	int i;
	/* SystemFrequency / 10000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if (SysTick_Config(SystemCoreClock / 10000)) // ST3.5.0��汾
	{
		while (1)
			;
	}
	// Enable SysTick
	SysTick->CTRL |= ~SysTick_CTRL_ENABLE_Msk;
	for (i = 0; i < TIMER_NUM; i++)
	{
		TimePre[i] = 0;
		TimeCur[i] = 0;
		TimerFlag[i] = 0;
	}
}

/*------------- Delay ms-----------------------------------------------------------*/
void Delay_ms(__IO u32 nTime)
{
	TimingDelay = nTime;
	while (TimingDelay != 0)
		;
}

/*------SysTick Handler -----------------------------------------------------------*/
void SysTick_Handler(void)
{
	int i;

	ModbusTimer();
	nlTicks++;

	if (!TimingDelay)
		TimingDelay--;

	for (i = 0; i < TIMER_NUM; i++)
	{
		if (TimeCur[i] > 1)
		{
			TimeCur[i]--;
		}
	}
}

/*------SysTick Timer init--------------------------------------------------------------*/
void SetTimer(u8 no, u16 val)
{
	if (no < TIMER_NUM)
	{
		TimePre[no] = val + 1;
		TimeCur[no] = val + 1;
	}
}

/*------SysTick Handler --------------------------------------------------------------*/
u16 GetTimer(u8 no)
{

	if (no < TIMER_NUM && TimeCur[no] == 1)
	{
		TimeCur[no] = TimePre[no];
		return 1;
	}

	return 0;
}

/*---------------GetCurTick()---------------------------------------------------------*/
u32 GetCurTick(void)
{
	return nlTicks;
}

/************************END OF FILE************/
