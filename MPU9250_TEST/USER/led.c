//����Ǳ��ļ�ֻ��Ӧ�Լ���ͷ�ļ��������ܵظ��ھ�
#include"led.h"
//����Ϊʲô����ӡ�gpio.h������ļ��أ���Ϊ��led.h�ļ��������stm32f10x.h�ļ�������ļ��а���conf.h�ļ�
//����������conf.h�ļ��������˶�Ӧ�������ļ�������һ���������ú�conf.h��ֱ�����stm32f10x.h���ɡ�
void LED_GPIO_Config()
{
	

	//gpio ��ʼ��
	//����3.5��İ����ļ����鿴��modules-gpio-datastructures ��֪���gpio��ʼ���ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	 //���ȿ�������ʱ��,����ʱ�ӿ���������modules-rcc-functions
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE );
	
	//�����GPIO_InitStructure����ṹ��ĳ�Ա��ʼ��
	//step1 ʵ��һ��GPIO_InitTypeDef�Ľṹ��
	//step2 ������ṹ��Ա��Ҫ������
	//step3 ���ÿ��ʼ�����������������þ��嵽����gpio��
	//step4 ���Ժ�Ϳ��������gpio��
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
