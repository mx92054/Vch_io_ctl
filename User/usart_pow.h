#ifndef __POW_USART__
#define __POW_USART__

#include "stm32f4xx.h"

//  COM1 Define

#define USART_POW USART6

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define POW_USART_CLK RCC_APB2Periph_USART6
#define POW_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define POW_USART_BAUDRATE 9600 //串口波特率

#define POW_USART_RX_GPIO_PORT GPIOC
#define POW_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define POW_USART_RX_PIN GPIO_Pin_6
#define POW_USART_RX_AF GPIO_AF_USART6
#define POW_USART_RX_SOURCE GPIO_PinSource6

#define POW_USART_TX_GPIO_PORT GPIOC
#define POW_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define POW_USART_TX_PIN GPIO_Pin_7
#define POW_USART_TX_AF GPIO_AF_USART6
#define POW_USART_TX_SOURCE GPIO_PinSource7

#define POW_USART_IRQ USART6_IRQn
#define POW_USART_IRQHandler USART6_IRQHandler

//--------------------------------------------------------------------
#define POW_NUM 5              //電源板數目
#define POW_SAVE_ADR 0         // POWE传感器参数在wReg中的起始地址
#define POW_BAUDRATE wReg[103] //通信波特率
#define POW_COM_TIM 70         //通信成功间隔寄存器地址
#define POW_COM_SUCS 80        //通信成功次数寄存器地址
#define POW_COM_FAIL 90        //通信失败次数寄存器地址

void POW_Init(void);
void POW_Task(void);

void POW_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
