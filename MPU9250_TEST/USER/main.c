//=================================================================================
//	Project_Name:	 SysTick
//	Decription:		learn how to use the systick 
//	Date:		  2012��9��15��21:22:56
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
//�������õ���systick�жϣ�����systick��Initֻ�ǵ�����SysTick_Config()���������������ں˲��Cortex-M3ͨ�ú�����
//λ��core_cm3.h�ļ��� ���������������SysTick timer������������Ϊ������0ʱ�����жϣ�����Ĳ���ticksΪ�����ж�֮�����������
//�����ticks��ʱ�����ڻ�����һ���жϣ�����SysTick�ɹ�ʱ����0�����������1�� ���ƶ���systick��ʱ��ѡ��
//Ҳ����˵systick-init������������systick��صĹ��������������жϡ�����ֻ�����жϺ�������Լ��Ĵ���ͺ���
#include "stm32f10x.h"
//#include"stm32f10x_conf.h" //as this .h haven been include in stm32f10x.h
#include"led.h"
#include"systick.h"
//SysTick���ж������ļ�core_cm3.h�ĺ������õģ�û��ʹ��NVIC�������жϣ����Կɲ����misc.c�ļ� ��
//misc.c�ļ�����������nvic�жϵ�
//��core_cm3.h�ڰ���stm32f10x.hͷ�ļ�ʱ�ѱ���ӽ������ˡ�
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
