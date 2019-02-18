/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ��1.5.1�汾�⽨�Ĺ���ģ��
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

	SysTick_Init();												  //tick��ʱ����ʼ
	GPIO_Config();												  //GPIO��ʼ��
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //ͨ�żĴ�����ʼ��
	wReg[102]++;												  //����������1
	bSaved = 1;													  //���浽EPROM

	Modbus_init(); //��λ��ͨ�ų�ʼ��
	POW_Init();	//��Դ��ͨ�ų�ʼ��
	SWB_Init();	//�̵�����ͨ�ų�ʼ��
	TLT_Init();	//��̨���ư�ͨ�ų�ʼ��
	INS_Init();	//��Ե���ͨ�ų�ʼ��

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);
	SetTimer(3, 100);

	IWDG_Configuration(); //���Ź���ʼ

	while (1)
	{
		Modbus_task(); //ͨ�ų�������
		POW_Task();
		SWB_Task();
		TLT_Task();
		INS_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //���Ź���λ
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //�����޸Ĺ��ļĴ���
			bSaved = 0;
		}

		if (GetTimer(3))
		{
			POW_TxCmd();
			SWB_TxCmd(); //�̵����巢��ȡָ��
			TLT_TxCmd(); //��3#����������ȡָ��
			INS_TxCmd(); //��ģ��������巢��ָ��
		}
	}
}

/*********************************************END OF FILE**********************/
