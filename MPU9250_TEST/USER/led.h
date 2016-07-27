#ifndef _LED_H
#define _LED_H
#include"stm32f10x.h"

//三个灯全部灭和亮
void LED_ON(void);
void LED_OFF(void);
//LED1 on off
void LED1_ON(void);
void LED1_OFF(void);
//LED2 on off
void LED2_ON(void);
void LED2_OFF(void);
//LED3 on  off
void LED3_ON(void);
void LED3_OFF(void);

void LED_GPIO_Config(void);

#endif 
