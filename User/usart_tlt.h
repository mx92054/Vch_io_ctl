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
#define TLT_BAUDRATE wReg[111]  //2#编码器通信速度
#define TLT_STATION wReg[112]   //2#编码器站地址
#define TLT_START_ADR wReg[113] //2#编码器参数首地址
#define TLT_REG_LEN wReg[114]   //2#编码器参数长度

#define TLT_CUR_ANG wReg[20]  //2#编码器当前角度
#define TLT_CUR_TICK wReg[21] //2#编码器当前角度
#define TLT_CUR_DETA wReg[22] //2#编码器当前角度
#define TLT_CUR_SPD wReg[23]  //2#编码器当前角度
#define TLT_LST_ANG wReg[24]  //2#编码器当前角度
#define TLT_LST_TICK wReg[25] //2#编码器当前角度
#define TLT_LST_DETA wReg[26] //2#编码器当前角度
#define TLT_AVG_SPD wReg[27]  //2#编码器当前角度
#define TLT_COM_FAIL wReg[28] //2#编码器当前角度
#define TLT_COM_SUCS wReg[29] //2#编码器当前角度

void TLT_Init(void);
void TLT_TxCmd(void);
void TLT_Task(void);

void TLT_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
