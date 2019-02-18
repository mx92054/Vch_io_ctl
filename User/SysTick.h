#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f4xx.h"

void SysTick_Init(void);
void Delay_ms(__IO u32 nTime);
void SysTick_Handler(void);
u16 GetTimer(u8 no);
void SetTimer(u8 no, u16 val);
u32 GetCurTick(void);

#endif /* __SYSTICK_H */
