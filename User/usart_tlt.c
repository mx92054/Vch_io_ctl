#include "usart_tlt.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern short wReg[];

uint8_t TLT_frame[21] = {0xFF, 0xFF, 0xA5, 0x40,
                         0x11, 0x11, 0x22, 0x22,
                         0x33, 0x33, 0x44, 0x44,
                         0x55, 0x55, 0x66, 0x66,
                         0xBC, 0xBC, 0xBC, 0xCC, 0x26};
u8 TLT_buffer[256];
u8 TLT_curptr;
u8 TLT_bRecv;
u8 TLT_frame_len = 85;
u8 TLT_bFirst = 1;
u32 ulTLTTick = 0;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void TLT_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = TLT_USART_IRQ;
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
static void TLT_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(TLT_USART_RX_GPIO_CLK | TLT_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    TLT_USART_APBxClkCmd(TLT_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = TLT_USART_TX_PIN;
    GPIO_Init(TLT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = TLT_USART_RX_PIN;
    GPIO_Init(TLT_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(TLT_USART_RX_GPIO_PORT, TLT_USART_RX_SOURCE, TLT_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(TLT_USART_TX_GPIO_PORT, TLT_USART_TX_SOURCE, TLT_USART_TX_AF);

    /* 配置串TLT_USART 模式 */
    /* 波特率设置：TLT_USART_BAUDRATE */
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
    USART_Init(USART_TLT, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    TLT_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_TLT, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_TLT, ENABLE);
}

/****************************************************************
 *	@brief:	    TLT通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void TLT_Init(void)
{
    if (TLT_BAUDRATE != 96 && TLT_BAUDRATE != 192 && TLT_BAUDRATE != 384 && TLT_BAUDRATE != 1152)
    {
        TLT_BAUDRATE = 384;
    }
    TLT_Config(TLT_BAUDRATE);

    TLT_frame_len = 14;
    TLT_curptr = 0;
    TLT_bRecv = 0;
    TLT_COM_FAIL = 0;
    ulTLTTick = GetCurTick();
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void TLT_TxCmd(void)
{
    u16 i;

    if (TLT_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        TLT_COM_FAIL++;

    TLT_curptr = 0;
    TLT_bRecv = 1;

    TLT_frame[4] = wReg[TLT_REG_ADR];
    TLT_frame[5] = wReg[TLT_REG_ADR + 1];
    TLT_frame[6] = wReg[TLT_REG_ADR + 2];
    TLT_frame[7] = wReg[TLT_REG_ADR + 3];

    TLT_frame[16] = wReg[TLT_REG_ADR + 4];
    TLT_frame[17] = wReg[TLT_REG_ADR + 5];
    TLT_frame[18] = wReg[TLT_REG_ADR + 6];

    TLT_frame[19] = TLT_frame[3];
    for (i = 4; i < 18; i++)
        TLT_frame[19] ^= TLT_frame[i];

    Usart_SendBytes(USART_TLT, TLT_frame, 21);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void TLT_Task(void)
{
    u8 *ptr, i;
    u32 tick;

    if (TLT_curptr < TLT_frame_len) // 未收到完整的數據幀
        return;

    if (TLT_buffer[0] != 0xFF || TLT_buffer[1] != 0xFF || TLT_buffer[2] != 0xA5)
        return; //幀格式錯誤

    if (TLT_buffer[3] != 0x40)
        return; //地址错误

    ptr = TLT_buffer + 4;
    for (i = 0; i < 4; i++)
    {
        wReg[TLT_REG_VAL + i] = *ptr++ << 8;
        wReg[TLT_REG_VAL + i] |= *ptr++;
    }

    tick = GetCurTick();
    TLT_COM_TCK = tick - ulTLTTick;
    ulTLTTick = tick;

    TLT_COM_SUCS++;
    TLT_bRecv = 0;
    TLT_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void TLT_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_TLT, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_TLT); //将读寄存器的数据缓存到接收缓冲区里
        TLT_buffer[TLT_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_TLT, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_TLT, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
