#include "usart_pow.h"
#include "Modbus_svr.h"
#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "SysTick.h"

extern short wReg[];

u8 POW_Txbuf[8] = {0xFF, 0xFF, 0xA5, 0x61, 0x01, 0x05, 0xCC, 0x26};
u8 POW_Swbuf[9] = {0xFF, 0xFF, 0xA5, 0x61, 0x02, 0x06, 0x00, 0xCC, 0x26};
char POW_buffer[256];
u8 POW_curptr;
u8 POW_bRecv;

u8 POW_Frame_len = 100;
u8 uCurPowNo = 0;				 //当前正在查询的电源板号
short uPowerStatus[2 * POW_NUM]; //保存上次电源的开关状态
u32 ulPowTicks[POW_NUM];

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void POW_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 嵌套向量中断控制器组选择 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = POW_USART_IRQ;
	/* 抢断优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* 子优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	/* 使能中断 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 初始化配置NVIC */
	NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void POW_Config(short baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(POW_USART_RX_GPIO_CLK | POW_USART_TX_GPIO_CLK, ENABLE);

	/* 使能 USART 时钟 */
	RCC_APB2PeriphClockCmd(POW_USART_CLK, ENABLE);

	/* GPIO初始化 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* 配置Tx引脚为复用功能  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = POW_USART_TX_PIN;
	GPIO_Init(POW_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* 配置Rx引脚为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = POW_USART_RX_PIN;
	GPIO_Init(POW_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(POW_USART_RX_GPIO_PORT, POW_USART_RX_SOURCE, POW_USART_RX_AF);

	/*  连接 PXx 到 USARTx__Rx*/
	GPIO_PinAFConfig(POW_USART_TX_GPIO_PORT, POW_USART_TX_SOURCE, POW_USART_TX_AF);

	/* 配置串POW_USART 模式 */
	/* 波特率设置：POW_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = baud * 100;
	/* 字长(数据位+校验位)：8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* 停止位：1个停止位 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* 校验位选择：不使用校验 */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* 硬件流控制：不使用硬件流 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USART模式控制：同时使能接收和发送 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* 完成USART初始化配置 */
	USART_Init(USART_POW, &USART_InitStructure);

	/* 嵌套向量中断控制器NVIC配置 */
	POW_NVIC_Configuration();

	/* 使能串口接收中断 */
	USART_ITConfig(USART_POW, USART_IT_RXNE, ENABLE);

	/* 使能串口 */
	USART_Cmd(USART_POW, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void POW_Init(void)
{
	int i;

	if (POW_BAUDRATE != 96 && POW_BAUDRATE != 192 && POW_BAUDRATE != 384 && POW_BAUDRATE != 1152)
	{
		POW_BAUDRATE = 384;
	}
	POW_Config(POW_BAUDRATE);

	for (i = 0; i < 2 * POW_NUM; i++)
	{
		uPowerStatus[i] = 0;
		ulPowTicks[i / 2] = GetCurTick();
	}

	POW_curptr = 0;
	POW_bRecv = 0;
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void POW_TxCmd(void)
{
	int adr1, adr2;

	if (POW_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
	{
		wReg[POW_COM_FAIL + uCurPowNo]++;
	}

	POW_curptr = 0;
	POW_bRecv = 1;

	adr1 = 2 * uCurPowNo;
	adr2 = POW_SW_ADR + adr1;
	if (wReg[adr2] != uPowerStatus[adr1] ||
		wReg[adr2 + 1] != uPowerStatus[adr1 + 1])
	{
		POW_Swbuf[3] = 0x61 + uCurPowNo;					 //电源板站号
		POW_Swbuf[6] = (wReg[adr2] == 0) ? 0x00 : 0x01;		 //通道1开关
		POW_Swbuf[6] |= (wReg[adr2 + 1] == 0) ? 0x00 : 0x10; //通道2开关
		POW_Swbuf[7] = POW_Swbuf[3] + POW_Swbuf[4] +		 //checksum
					   POW_Swbuf[5] + POW_Swbuf[6];			 //checksum
		Usart_SendBytes(USART_POW, POW_Swbuf, 9);
		POW_Frame_len = 9;
	}
	else
	{															   //电源板状态查询
		POW_Txbuf[3] = 0x61 + uCurPowNo;						   //address
		POW_Txbuf[6] = POW_Txbuf[3] + POW_Txbuf[4] + POW_Txbuf[5]; //checksum
		Usart_SendBytes(USART_POW, POW_Txbuf, 8);
		POW_Frame_len = 29;
	}

	uCurPowNo = (uCurPowNo + 1) % POW_NUM; //board address + 1
}
//-------------------------------------------------------------------------------
//	@brief	接收数据处理
//	@param	None
//	@retval	None
//   Frame format 1: $ISPOW,-000.485,M,000.9689,B,30.81,C*28
//   Frame format 2: $ISHPR,060.8,-89.5,-081.1*60
//-------------------------------------------------------------------------------
void POW_Task(void)
{
	int i;
	u8 nStat;
	char *ptr;
	short *pVal;
	u32 tick;

	if (POW_curptr < POW_Frame_len) // 未收到完整的數據幀
		return;

	if (POW_buffer[0] != 0xFF || POW_buffer[1] != 0xFF || POW_buffer[2] != 0xA5)
		return; //幀格式錯誤

	nStat = POW_buffer[3] - 0x61;
	if (nStat >= POW_NUM)
		return; //地址錯誤

	if (POW_buffer[4] == 0x81) //是讀取電源板參數命令
	{
		tick = GetCurTick();
		ptr = POW_buffer + 6;
		pVal = &wReg[POW_SAVE_ADR + nStat * 6];
		for (i = 0; i < 6; i++)
		{
			*pVal = (*ptr++) << 8;
			*pVal |= *ptr++;
			pVal++;
		}
		wReg[POW_COM_SUCS + nStat]++;
		wReg[POW_COM_TIM + nStat] = tick - ulPowTicks[nStat];
		ulPowTicks[nStat] = tick;
	}

	if (POW_buffer[4] == 0x82) //是控制电源开关指令
	{
		uPowerStatus[2 * nStat] = wReg[POW_SW_ADR + 2 * i];
		uPowerStatus[2 * nStat + 1] = wReg[POW_SW_ADR + 2 * i + 1];
	}

	POW_bRecv = 0;
}

/*******************************************************************************
 *	@brief	串口中断服务程序
 *	@param	None
 *	@retval	None
 ********************************************************************************/
void POW_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(USART_POW, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
	{
		ch = USART_ReceiveData(USART_POW); //将读寄存器的数据缓存到接收缓冲区里
		POW_buffer[POW_curptr++] = ch;
	}

	if (USART_GetITStatus(USART_POW, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(USART_POW, USART_IT_TXE, DISABLE);
	}
}

//-----------------------------End of file--------------------------------------------------
