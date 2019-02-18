#ifndef __MODBUS_COM__
#define __MODBUS_COM__

#include "stm32f4xx.h"
#include <stdio.h>

#define USE_USART1

//引脚定义
/*******************************************************/
#ifdef USE_USART1

#define DEBUG_USARTx USART1

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define DEBUG_USART_CLK RCC_APB2Periph_USART1
#define DEBUG_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define DEBUG_USART_BAUDRATE 115200 //串口波特率

#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_PIN GPIO_Pin_10
#define DEBUG_USART_RX_AF GPIO_AF_USART1
#define DEBUG_USART_RX_SOURCE GPIO_PinSource10

#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN GPIO_Pin_9
#define DEBUG_USART_TX_AF GPIO_AF_USART1
#define DEBUG_USART_TX_SOURCE GPIO_PinSource9

#define DEBUG_USART_IRQ USART1_IRQn
#define DEBUG_USART_IRQHandler USART1_IRQHandler

#endif
/************************************************************/

//-------------------串口2宏定义------------------------------------
#ifdef USE_USART2
// 串口1-USART1
#define DEBUG_USARTx USART2
#define DEBUG_USART_CLK RCC_APB1Periph_USART2
#define DEBUG_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define DEBUG_USART_BAUDRATE 115200

// USART GPIO 引脚宏定义
#define DEBUG_USART_GPIO_CLK (RCC_APB2Periph_GPIOA)
#define DEBUG_USART_GPIO_APBxClkCmd RCC_APB2PeriphClockCmd

#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_PIN GPIO_Pin_2
#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_PIN GPIO_Pin_3

#define DEBUG_USART_IRQ USART2_IRQn
#define DEBUG_USART_IRQHandler USART2_IRQHandler
#endif

//-------------------MODBUS DATA DEFINE--------------------------------
#define REG_LEN 200
#define COIL_LEN 200
#define BIT2BYTE(n) ((((n)&0x0007) == 0) ? ((n) >> 3) : (((n) >> 3) + 1))
#define SETBIT_BYTE(n, bit) ((n) | (0x01 << (bit)))
#define RESETBIT_BYTE(n, bit) ((n) & (~(0x01 << (bit))))
#define GETBIT_BYTE(n, bit) (((n) >> (bit)) & 0x01)

extern u8 bSaved;

//------------------Function Define ----------------------------------
void Modbus_init(void);
void Modbus_task(void);
u8 Modbus_Procotol_Chain(void);
void ModbusTimer(void);
u16 CRC16(const uint8_t *nData, uint8_t wLength);
void DEBUG_USART_IRQHandler(void);
void SaveToBKP(u16 nAddr, u16 val);
void Error_response(u8 errno);
void Normal_response(void);

void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch);
void Usart_SendBytes(USART_TypeDef *pUSARTx, uint8_t *buf, uint8_t len);
void Usart_SendString(USART_TypeDef *pUSARTx, char *str);
void Usart_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch);

#endif
//------------end fo file----------------------------------
