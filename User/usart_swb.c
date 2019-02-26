#include "usart_swb.h"
#include "spd_comm.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern short wReg[];

u8 SWB_buffer[256];
u8 SWB_curptr;
u8 SWB_bRecv;
u8 SWB_frame_len = 16;
u8 SWB_bFirst = 1;
u32 ulSWBTick = 0;

u8 SWB_DOB_buf[10] = {0xFF, 0xFF, 0xA5, 0x30, 0x02, 0x06, 0x00, 0x00, 0xCC, 0x26}; //继电器板命令格式
u8 SWB_TMP_buf[8] = {0xFF, 0xFF, 0xA5, 0x32, 0x01, 0x05, 0xCC, 0x26};              //温度板命令格式
u8 SWB_INS_buf[8] = {0xFF, 0xFF, 0xA5, 0x31, 0x01, 0x05, 0xCC, 0x26};              //漏水板命令格式
u8 curDOB = 0;                                                                     //当前输出的继电器板
u8 bComTmp = 1;
short sCurStatus[3];             //当前需要方面温度板
short sLstStatus[3] = {0, 0, 0}; //当前开关量状态
u32 ulTmpTick = 0;
u32 ulLekTick = 0;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SWB_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = SWB_USART_IRQ;
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
static void SWB_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(SWB_USART_RX_GPIO_CLK | SWB_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    SWB_USART_APBxClkCmd(SWB_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SWB_USART_TX_PIN;
    GPIO_Init(SWB_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SWB_USART_RX_PIN;
    GPIO_Init(SWB_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(SWB_USART_RX_GPIO_PORT, SWB_USART_RX_SOURCE, SWB_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(SWB_USART_TX_GPIO_PORT, SWB_USART_TX_SOURCE, SWB_USART_TX_AF);

    /* 配置串SWB_USART 模式 */
    /* 波特率设置：SWB_USART_BAUDRATE */
    USART_InitStructure.USART_BaudRate = wBaudrate * 100;
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
    USART_Init(USART_SWB, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    SWB_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_SWB, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_SWB, ENABLE);
}

/****************************************************************
 *	@brief:	    SWB通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void SWB_Init(void)
{
    if (SWB_BAUDRATE != 96 && SWB_BAUDRATE != 192 && SWB_BAUDRATE != 384 && SWB_BAUDRATE != 1152)
    {
        SWB_BAUDRATE = 384;
    }
    SWB_Config(SWB_BAUDRATE);

    SWB_curptr = 0;
    SWB_bRecv = 0;
    ulSWBTick = GetCurTick();

    SWB_TMP_buf[6] = SWB_TMP_buf[3] ^ SWB_TMP_buf[4] ^ SWB_TMP_buf[5];
    SWB_INS_buf[6] = SWB_INS_buf[3] ^ SWB_INS_buf[4] ^ SWB_INS_buf[5];
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SWB_TxCmd(void)
{
    int i;

    if (SWB_bRecv == 1 && bComTmp == 0) //如果当前未完成接收，则通信错误计数器递增
        wReg[SWB_TMP_FAIL]++;

    if (SWB_bRecv == 1 && bComTmp == 1) //如果当前未完成接收，则通信错误计数器递增
        wReg[SWB_LEK_FAIL]++;

    SWB_curptr = 0;
    SWB_bRecv = 1;

    curDOB = (curDOB + 1) % 3;
    sCurStatus[curDOB] = 0;
    for (i = 0; i < 9; i++)
    {
        if (wReg[SWB_DOB_ADR + 9 * curDOB + i] != 0)
            sCurStatus[curDOB] |= (0x0001 << i);
    }

    if (sCurStatus[curDOB] != sLstStatus[curDOB]) //继电器状态发生改变
    {
        if (curDOB == 0)
            SWB_DOB_buf[3] = 0x30;
        if (curDOB == 1)
            SWB_DOB_buf[3] = 0x29;
        if (curDOB == 2)
            SWB_DOB_buf[3] = 0x28;
        SWB_DOB_buf[6] = sCurStatus[curDOB] & 0x00FF;
        SWB_DOB_buf[7] = (sCurStatus[curDOB] & 0xFF00) >> 8;
        SWB_DOB_buf[8] = SWB_DOB_buf[3] ^ SWB_DOB_buf[4] ^
                         SWB_DOB_buf[5] ^ SWB_DOB_buf[6] ^ SWB_DOB_buf[7];
        Usart_SendBytes(USART_SWB, SWB_DOB_buf, 10);
        SWB_frame_len = 8;

        wReg[170] = sCurStatus[curDOB];
        wReg[171] = sLstStatus[curDOB];
        wReg[172] = curDOB;

        return;
    }

    if (bComTmp) //读取温度状态
    {
        Usart_SendBytes(USART_SWB, SWB_TMP_buf, 8);
        SWB_frame_len = 16;
    }
    else //读取漏水状态
    {
        Usart_SendBytes(USART_SWB, SWB_INS_buf, 8);
        SWB_frame_len = 10;
    }

    bComTmp = (bComTmp + 1) % 2;
    SWB_curptr = 0;
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void SWB_Task(void)
{
    int i;
    u8 *ptr;
    u32 tick;

    if (SWB_curptr < SWB_frame_len) // 未收到完整的數據幀
        return;

    if (SWB_buffer[0] != 0xFF || SWB_buffer[1] != 0xFF || SWB_buffer[2] != 0xA5)
        return; //幀格式錯誤

    tick = GetCurTick();
    switch (SWB_buffer[3])
    {
    case 0x30: //1#继电器板
        wReg[171] = sCurStatus[0];
        sLstStatus[0] = sCurStatus[0];
        break;

    case 0x29: //2#继电器板
        wReg[171] = sCurStatus[1];
        sLstStatus[1] = sCurStatus[1];
        break;

    case 0x28: //3#继电器板
        wReg[171] = sCurStatus[2];
        sLstStatus[2] = sCurStatus[2];
        break;

    case 0x32: //温度板
        ptr = SWB_buffer + 4;
        for (i = 0; i < 5; i++)
        {
            wReg[SWB_TMP_ADR + i] = *ptr++ << 8;
            wReg[SWB_TMP_ADR + i] |= *ptr++;
        }
        wReg[SWB_TMP_TICK] = tick - ulTmpTick;
        ulTmpTick = tick;
        wReg[SWB_TMP_SUCS]++;
        break;

    case 0x31: //漏水板
        ptr = SWB_buffer + 4;
        wReg[SWB_LEK_ADR] = *ptr & 0x07;
        wReg[SWB_LEK_ADR + 1] = (*ptr >> 3) & 0x07;
        wReg[SWB_LEK_ADR + 2] = (*ptr >> 6) & 0x03;
        ptr++;
        wReg[SWB_LEK_ADR + 2] |= (*ptr & 0x01) << 2;
        wReg[SWB_LEK_ADR + 3] = *ptr >> 1 & 0x07;
        wReg[SWB_LEK_ADR + 4] = *ptr >> 4 & 0x07;
        wReg[SWB_LEK_ADR + 5] = *ptr >> 7;
        ptr++;
        wReg[SWB_LEK_ADR + 5] |= (*ptr & 0x03) << 1;
        wReg[SWB_LEK_ADR + 6] = (*ptr >> 2) & 0x07;
        wReg[SWB_LEK_TICK] = tick - ulLekTick;

        wReg[SWB_LEK_TICK] = tick - ulTmpTick;
        ulLekTick = tick;
        wReg[SWB_LEK_SUCS]++;
        break;
    }

    SWB_curptr = 0;
    SWB_bRecv = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
int SWB_Frame_len = 100;
void SWB_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_SWB, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_SWB); //将读寄存器的数据缓存到接收缓冲区里
        SWB_buffer[SWB_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_SWB, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_SWB, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
