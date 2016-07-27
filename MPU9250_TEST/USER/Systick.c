#include"systick.h"

//timingdelay这个变量只在本函数用到，所以定义为静态
//__IO  为volatile的宏定义，表示被修饰的变量调用时很严格，不可采取寄存器映射
//注意__IO写法。前面两个_ _连一块，后面i和o都要大写。
static __IO uint32_t TimingDelay;

void SysTick_Init(void)
{
	// SystemFrequency / 1000 1ms中断一次
	// SystemFrequency / 100000 10us中断一次
	// SystemFrequency / 1000000 1us中断一次
	//systick_config(ticks)这个函数是core_cm3中的文件
	//这个函数启动了SysTick timer；并把它配置为计数至0时引起中断；
	//输入的参数ticks为两个中断之间的脉冲数，即相隔ticks个时钟周期会引起一次中断；
	//配置SysTick成功时返回0，出错进返回1
	if(SysTick_Config(SystemCoreClock/1000000))
	{
		while(1);
	}
	//先关闭SysTick定时器
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}
void Delay_us(__IO uint32_t nTime)
{
	TimingDelay=nTime;

	//使能嘀嗒定时器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay!=0);
	//TimingDelay这个变量是全局变量，在it。c中断服务函数中自减的
}
//下面这个函数式被it.c中断服务程序调用的，当然也可以定在it.c文件中。
void TimingDelay_Decrement(void)
{
	if(TimingDelay !=0x00)
	{
		TimingDelay--;
	}
}
