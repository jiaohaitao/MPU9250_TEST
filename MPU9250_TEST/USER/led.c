//最好是本文件只对应自己的头文件，尽可能地高内聚
#include"led.h"
//这里为什么不添加“gpio.h”这个文件呢？因为在led.h文件中添加了stm32f10x.h文件，这个文件中包含conf.h文件
//而我们又在conf.h文件里配置了对应的外设文件，所以一旦我们配置好conf.h后，直接添加stm32f10x.h即可。
void LED_GPIO_Config()
{
	

	//gpio 初始化
	//查阅3.5库的帮助文件，查看：modules-gpio-datastructures 可知库的gpio初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	 //首先开启外设时钟,外设时钟开启函数在modules-rcc-functions
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE );
	
	//下面对GPIO_InitStructure这个结构体的成员初始化
	//step1 实例一个GPIO_InitTypeDef的结构体
	//step2 对上面结构成员按要求配置
	//step3 调用库初始化函数，将上面配置具体到摸个gpio口
	//step4 这以后就可以用这个gpio了
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;

	GPIO_InitStructure.GPIO_Pin=(GPIO_Pin_13);

	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz ;  

	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits (GPIOC,GPIO_Pin_13);
		
}
void LED_OFF()
{
	GPIO_SetBits (GPIOC,GPIO_Pin_13);
}
void LED_ON()
{
	GPIO_ResetBits (GPIOC,GPIO_Pin_13);
}

void LED1_OFF()
{
  GPIO_SetBits (GPIOC,GPIO_Pin_13);
}
void LED1_ON()
{
  GPIO_ResetBits (GPIOC,GPIO_Pin_13);
}
