#include"systick.h"

//timingdelay�������ֻ�ڱ������õ������Զ���Ϊ��̬
//__IO  Ϊvolatile�ĺ궨�壬��ʾ�����εı�������ʱ���ϸ񣬲��ɲ�ȡ�Ĵ���ӳ��
//ע��__IOд����ǰ������_ _��һ�飬����i��o��Ҫ��д��
static __IO uint32_t TimingDelay;

void SysTick_Init(void)
{
	// SystemFrequency / 1000 1ms�ж�һ��
	// SystemFrequency / 100000 10us�ж�һ��
	// SystemFrequency / 1000000 1us�ж�һ��
	//systick_config(ticks)���������core_cm3�е��ļ�
	//�������������SysTick timer������������Ϊ������0ʱ�����жϣ�
	//����Ĳ���ticksΪ�����ж�֮����������������ticks��ʱ�����ڻ�����һ���жϣ�
	//����SysTick�ɹ�ʱ����0�����������1
	if(SysTick_Config(SystemCoreClock/1000000))
	{
		while(1);
	}
	//�ȹر�SysTick��ʱ��
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}
void Delay_us(__IO uint32_t nTime)
{
	TimingDelay=nTime;

	//ʹ����શ�ʱ��
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay!=0);
	//TimingDelay���������ȫ�ֱ�������it��c�жϷ��������Լ���
}
//�����������ʽ��it.c�жϷ��������õģ���ȻҲ���Զ���it.c�ļ��С�
void TimingDelay_Decrement(void)
{
	if(TimingDelay !=0x00)
	{
		TimingDelay--;
	}
}
