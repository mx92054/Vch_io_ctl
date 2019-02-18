/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   用1.5.1版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "SysTick.h"
#include "Modbus_svr.h"
#include "usart_pow.h"
#include "usart_swb.h"
#include "usart_tlt.h"
#include "usart_ins.h"
#include "gpio.h"
#include "bsp_innerflash.h"

extern short wReg[];
extern u8 bChanged;

int main(void)
{

	SysTick_Init();												  //tick定时器初始
	GPIO_Config();												  //GPIO初始化
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //通信寄存器初始化
	wReg[102]++;												  //启动次数加1
	bSaved = 1;													  //保存到EPROM

	Modbus_init(); //上位机通信初始化
	POW_Init();	//电源板通信初始化
	SWB_Init();	//继电器板通信初始化
	TLT_Init();	//云台控制板通信初始化
	INS_Init();	//绝缘检测通信初始化

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);
	SetTimer(3, 100);

	IWDG_Configuration(); //看门狗初始

	while (1)
	{
		Modbus_task(); //通信出来进程
		POW_Task();
		SWB_Task();
		TLT_Task();
		INS_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //看门狗复位
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //保存修改过的寄存器
			bSaved = 0;
		}

		if (GetTimer(3))
		{
			POW_TxCmd();
			SWB_TxCmd(); //继电器板发读取指令
			TLT_TxCmd(); //向3#编码器发读取指令
			INS_TxCmd(); //向模拟量输出板发出指令
		}
	}
}

/*********************************************END OF FILE**********************/
