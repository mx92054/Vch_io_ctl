#ifndef __SWB_USART__
#define __SWB_USART__

#include "stm32f4xx.h"

//COM2 Define

#define USART_SWB USART3

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SWB_USART_CLK RCC_APB1Periph_USART3
#define SWB_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SWB_USART_BAUDRATE 38400 //串口波特率

#define SWB_USART_RX_GPIO_PORT GPIOB
#define SWB_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define SWB_USART_RX_PIN GPIO_Pin_10
#define SWB_USART_RX_AF GPIO_AF_USART3
#define SWB_USART_RX_SOURCE GPIO_PinSource10

#define SWB_USART_TX_GPIO_PORT GPIOB
#define SWB_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define SWB_USART_TX_PIN GPIO_Pin_11
#define SWB_USART_TX_AF GPIO_AF_USART3
#define SWB_USART_TX_SOURCE GPIO_PinSource11

#define SWB_USART_IRQ USART3_IRQn
#define SWB_USART_IRQHandler USART3_IRQHandler

//----------------------------------------------------------------
#define SWB_BAUDRATE wReg[107]  //1#编码器通信速度
#define SWB_STATION wReg[108]   //1#编码器站地址
#define SWB_START_ADR wReg[109] //1#编码器参数首地址
#define SWB_REG_LEN wReg[110]   //1#编码器参数长度

#define SWB_CUR_ANG wReg[10]  //1#编码器当前角度
#define SWB_CUR_TICK wReg[11] //1#编码器当前角度
#define SWB_CUR_DETA wReg[12] //1#编码器当前角度
#define SWB_CUR_SPD wReg[13]  //1#编码器当前角度
#define SWB_LST_ANG wReg[14]  //1#编码器当前角度
#define SWB_LST_TICK wReg[15] //1#编码器当前角度
#define SWB_LST_DETA wReg[16] //1#编码器当前角度
#define SWB_AVG_SPD wReg[17]  //1#编码器当前角度
#define SWB_COM_FAIL wReg[18] //1#编码器当前角度
#define SWB_COM_SUCS wReg[19] //1#编码器当前角度

void SWB_Init(void);
void SWB_TxCmd(void);
void SWB_Task(void);

void SWB_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
