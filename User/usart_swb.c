#include "usart_swb.h"
#include "spd_comm.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern u16 wReg[];

uint8_t SWB_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
u8 SWB_buffer[256];
u8 SWB_curptr;
u8 SWB_bRecv;
u8 SWB_frame_len = 85;
u8 SWB_bFirst = 1 ;
u32 ulSWBTick = 0;

SpeedValueQueue qSWB;

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
    SWB_COM_FAIL = 0;
    SWB_frame_len = 2 * SWB_REG_LEN + 5;
    ulSWBTick = GetCurTick();

    SpdQueueInit(&qSWB);
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SWB_TxCmd(void)
{
    u16 uCRC;

    if (SWB_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        SWB_COM_FAIL++;

    SWB_curptr = 0;
    SWB_bRecv = 1;

    if (bChanged || SWB_bFirst)
    {
        SWB_frame[0] = SWB_STATION;                   //station number
        SWB_frame[2] = (SWB_START_ADR & 0xff00) >> 8; //start address high
        SWB_frame[3] = SWB_START_ADR & 0x00ff;        //start address low
        SWB_frame[4] = (SWB_REG_LEN & 0xff00) >> 8;   //length high
        SWB_frame[5] = SWB_REG_LEN & 0x00ff;          //length low
        uCRC = CRC16(SWB_frame, 6);
        SWB_frame[6] = uCRC & 0x00FF;        //CRC low
        SWB_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
        bChanged++;
        SWB_bFirst = 0;
    }

    Usart_SendBytes(USART_SWB, SWB_frame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void SWB_Task(void)
{
    u32 tick;

    if (SWB_curptr < SWB_frame_len)
        return;

    if (SWB_buffer[0] != SWB_STATION || SWB_buffer[1] != 0x03) //站地址判断
        return;

    if (SWB_buffer[2] != 2 * SWB_REG_LEN) //数值长度判读
        return;

    tick = GetCurTick();
    SWB_LST_ANG = SWB_CUR_ANG;   //上次编码器值
    SWB_LST_TICK = SWB_CUR_TICK; //上次计时器值
    SWB_LST_DETA = SWB_CUR_DETA; //上次角度变化值

    SWB_CUR_ANG = SWB_buffer[3] << 0x08 | SWB_buffer[4]; //本次编码器值
    SWB_CUR_TICK = tick - ulSWBTick;                      //本次计时器值
    ulSWBTick = tick;                                      //保存计时器
    SWB_CUR_DETA = SWB_CUR_ANG - SWB_LST_ANG;            //本次角度变化量
    if (SWB_CUR_ANG < 1024 && SWB_LST_ANG > 3072)
    {
        SWB_CUR_DETA = SWB_CUR_ANG - SWB_LST_ANG + 4096;
    }
    if (SWB_CUR_ANG > 3072 && SWB_LST_ANG < 1024)
    {
        SWB_CUR_DETA = SWB_CUR_ANG - SWB_LST_ANG - 4096;
    }
    if (SWB_CUR_TICK != 0)
        SWB_CUR_SPD = SWB_CUR_DETA * 1000 / SWB_CUR_TICK; //本次速度

    SpdQueueIn(&qSWB, SWB_CUR_DETA, SWB_CUR_TICK);
    SWB_AVG_SPD = SpdQueueAvgVal(&qSWB); //10次平均速度

    SWB_COM_SUCS++;
    SWB_bRecv = 0;
    SWB_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
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
