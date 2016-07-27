//=================================================================================
//	Project_Name:	 SysTick
//	Decription:		learn how to use the systick 
//	Date:		  2012年9月15日21:22:56
//	Made By:Haitao Jiao,computer engineering dept ,AYIT
//				You should love english and must.
//==================================================================================
//		MY STEP
//step0: rename the pjc name,revamp the "read me"
//step1: include rcc.h,gpio.h,led.h,systick.h & add files:led.c,systick.c
//		remove useless files of the lib(stm32f10x_ppp.h)
//step2: edit the systick.c
//step3: edit the it.c/SysTick_Handler(void) 
//step4: edit the main
//本例程用到了systick中断，首先systick—Init只是调用了SysTick_Config()函数，它是属于内核层的Cortex-M3通用函数，
//位于core_cm3.h文件中 ：这个函数启动了SysTick timer；并把它配置为计数至0时引起中断；输入的参数ticks为两个中断之间的脉冲数，
//即相隔ticks个时钟周期会引起一次中断；配置SysTick成功时返回0，出错进返回1。 并制定了systick的时钟选择
//也就是说systick-init（）函数做了systick相关的工作，并声明了中断。我们只需在中断函数添加自己的代码就好了
#include "stm32f10x.h"
//#include"stm32f10x_conf.h" //as this .h haven been include in stm32f10x.h
#include"led.h"
#include"systick.h"
//SysTick的中断是在文件core_cm3.h的函数配置的，没有使用NVIC来配置中断，所以可不添加misc.c文件 。
//misc.c文件是用来配置nvic中断的
//而core_cm3.h在包含stm32f10x.h头文件时已被添加进工程了。
uint32_t t=1000;
int main(void)
{
	//my code
	LED_GPIO_Config();
	SysTick_Init();
	while(1)
	{
		LED_ON();
		Delay_us(t);
		LED_OFF();
		Delay_us(t);
	}
}
