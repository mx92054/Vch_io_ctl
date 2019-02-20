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

	SetTimer(0, 100);  //看门狗定时器
	SetTimer(1, 1000); //保存参数定时器

	SetTimer(2, 100);  //电源板定时器
	SetTimer(3, 200);  //继电器板定时器
	SetTimer(4, 500);  //云台控制板定时器
	SetTimer(5, 1000); //绝缘检测板板定时器

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
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //保存修改过的寄存器
			bSaved = 0;
		}

		if (GetTimer(2))
			POW_TxCmd(); //电源控制板发读取指令
		if (GetTimer(3))
			SWB_TxCmd(); //继电器板发读取指令
		if (GetTimer(4))
			TLT_TxCmd(); //云台控制板发读取指令
		if (GetTimer(5))
			INS_TxCmd(); //绝缘检测板发出指令
	}
}

/*********************************************END OF FILE**********************/
