#ifndef _SYSTICK_H
#define _SYSTICK_H

#include"stm32f10x.h"

void SysTick_Init(void);
void Delay_us(uint32_t);
void TimingDelay_Decrement(void);

#endif
