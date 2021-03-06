#ifndef __TILT_USART__
#define __TILT_USART__

#include "stm32f4xx.h"

//COM3 Define

#define USART_TLT UART7

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define TLT_USART_CLK RCC_APB1Periph_UART7
#define TLT_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define TLT_USART_BAUDRATE 38400 //串口波特率

#define TLT_USART_RX_GPIO_PORT GPIOE
#define TLT_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define TLT_USART_RX_PIN GPIO_Pin_7
#define TLT_USART_RX_AF GPIO_AF_UART7
#define TLT_USART_RX_SOURCE GPIO_PinSource7

#define TLT_USART_TX_GPIO_PORT GPIOE
#define TLT_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define TLT_USART_TX_PIN GPIO_Pin_8
#define TLT_USART_TX_AF GPIO_AF_UART7
#define TLT_USART_TX_SOURCE GPIO_PinSource8

#define TLT_USART_IRQ UART7_IRQn
#define TLT_USART_IRQHandler UART7_IRQHandler

//----------------------------------------------------------------
#define TLT_BAUDRATE wReg[111] //云台通信速度

#define TLT_COM_TCK wReg[78]  //云台通信间隔时间
#define TLT_COM_SUCS wReg[88] //云台通信成功次数
#define TLT_COM_FAIL wReg[98] //云台通信失败次数

#define TLT_REG_ADR 160     //命令寄存器开始地址
#define TLT_REG_VAL 50      //状态寄存器开始地址

void TLT_Init(void);
void TLT_TxCmd(void);
void TLT_Task(void);

void TLT_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
