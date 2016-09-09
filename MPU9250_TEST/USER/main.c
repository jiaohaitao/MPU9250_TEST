#include "stm32f10x.h"
#include"led.h"
#include"systick.h"
#include "imu_init_position.h"
#include "mpu9250_driver.h"
#include "inv_imu_app.h"

#include "delay.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "stm32_iic.h"


#define BUFLEN 200

__IO unsigned char Usart2RxBuf[BUFLEN]={0};
__IO unsigned char Usart2RxOkFlag=0;
__IO unsigned int Usart2RxOkIndex=0;





float Imu_Init_Position[3];

void Data_Send_Status(float rol,float pit,float yaw);
void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9);
void USART1_Config(void);
void USART_NVIC_Configuration(void);
void USART2_Config(void);

unsigned char Calibration_Flag=0;
unsigned char Calibration_Ok_Flag=0;
void USART1_IRQHandler(void)
{
	unsigned char rxdata;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    rxdata=USART1->DR;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
		if(rxdata=='C'){
		Calibration_Flag=1;
		}
		else if(rxdata=='O'){
			Calibration_Ok_Flag=1;
			Calibration_Flag=0;
		}
	}
	
	 
} 

short MAG_MAX[3]={0};
short MAG_MIN[3]={0};
extern short IMU_MAG_OFFSET[3];

int main(void)
{
	int a_x_sum,a_y_sum,a_z_sum,g_x_sum,g_y_sum,g_z_sum,m_x_sum,m_y_sum,m_z_sum,i;
	float rol_sum,pit_sum,yaw_sum;
	short Acc[3],Mag[3],Gyo[3];
	MPU9250_Initial();
	USART1_Config();
	USART_NVIC_Configuration();
	//USART2_Config();
	
	while(1)
	{
#if 1
		while(Calibration_Flag==1){
			READ_MPU9250(Acc,Gyo,Mag);
			Send_Data(Acc[0],Acc[1],Acc[2],Gyo[0],Gyo[1],Gyo[2],Mag[0],Mag[1],Mag[2]);
			if(Mag[0]>MAG_MAX[0]){
				MAG_MAX[0]=Mag[0];
			}
			else if(Mag[0]<MAG_MIN[0]){
				MAG_MIN[0]=Mag[0];
			}
			else{}
				
			if(Mag[1]>MAG_MAX[1]){
				MAG_MAX[1]=Mag[1];
			}
			else if(Mag[1]<MAG_MIN[1]){
				MAG_MIN[1]=Mag[1];
			}
			else{}

			if(Mag[2]>MAG_MAX[2]){
				MAG_MAX[2]=Mag[2];
			}
			else if(Mag[2]<MAG_MIN[2]){
				MAG_MIN[2]=Mag[2];
			}
			else{}				
			
		}
		
		if(Calibration_Ok_Flag==1){
			Calibration_Ok_Flag=0;
			IMU_MAG_OFFSET[0]=(MAG_MAX[0]+MAG_MIN[0])/2;
			IMU_MAG_OFFSET[1]=(MAG_MAX[1]+MAG_MIN[1])/2;
			IMU_MAG_OFFSET[2]=(MAG_MAX[2]+MAG_MIN[2])/2;
		}
#endif		
		Get_Imu_Init_Position(Imu_Init_Position);
		Data_Send_Status(Imu_Init_Position[0],Imu_Init_Position[1],-1*Imu_Init_Position[2]);
	}
	
}
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
//	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(ad1);
	data_to_send[_cnt++]=BYTE0(ad1);
	data_to_send[_cnt++]=BYTE1(ad2);
	data_to_send[_cnt++]=BYTE0(ad2);
	data_to_send[_cnt++]=BYTE1(ad3);
	data_to_send[_cnt++]=BYTE0(ad3);
	
	data_to_send[_cnt++]=BYTE1(ad4);
	data_to_send[_cnt++]=BYTE0(ad4);
	data_to_send[_cnt++]=BYTE1(ad5);
	data_to_send[_cnt++]=BYTE0(ad5);
	data_to_send[_cnt++]=BYTE1(ad6);
	data_to_send[_cnt++]=BYTE0(ad6);
	data_to_send[_cnt++]=BYTE1(ad7);
	data_to_send[_cnt++]=BYTE0(ad7);
	data_to_send[_cnt++]=BYTE1(ad8);
	data_to_send[_cnt++]=BYTE0(ad8);
	data_to_send[_cnt++]=BYTE1(ad9);
	data_to_send[_cnt++]=BYTE0(ad9);
	
	data_to_send[3] = _cnt-4;
	//o¨ªD¡ê?¨¦
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++){
		USART_SendData(USART1,data_to_send[i]);
		while( USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );
	}
}

void Data_Send_Status(float rol,float pit,float yaw)
{
	u8 _cnt=0,data_to_send[100]={0};
	vs16 _temp;vs32 _temp2 = 0;	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
		
	
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++){
	USART_SendData(USART1,data_to_send[i]);
		while( USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );	
	}
	
}
void USART2_IRQHandler(void)
{
	static unsigned char i=0;
	static unsigned char sum=0;
	unsigned char rxdata;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
	    rxdata=USART2->DR;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

		i%=BUFLEN;
		Usart2RxBuf[i]=rxdata;
		//sum=sum-Usart1RxBuf[(i+BUFLEN-5)%BUFLEN]+Usart1RxBuf[(i+BUFLEN-1)%BUFLEN];
		if(Usart2RxBuf[(i+BUFLEN-20)%BUFLEN]==0x02&&Usart2RxBuf[(i+BUFLEN-21)%BUFLEN]==0xAA	\
			&&Usart2RxBuf[(i+BUFLEN-22)%BUFLEN]==0xAA)
		{
			Usart2RxOkFlag=1;
			Usart2RxOkIndex=i;
		}
		i++;		
	} 
	 
} 

    


void USART_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	

	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
}
void USART1_Config()
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART1, ENABLE);
}
//void USART2_Config(void)
//{
//	//gpio£¬usart³õÊ¼»¯½á¹¹Ìå¶¨Òå
//	//µ±gpio¸´ÓÃÊ±£¨¿ªÆôÍâÉèÊ±£©£¬ÐèÒª³õÊ¼»¯Í¬Ê±gpioºÍpppÍâÉè£¬²¢¶ÔËûÃÇÊ¹ÄÜºÍ¿ªÆôÊ±ÖÓ
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	
//	//¿ªÆôgpioºÍÍâÉèÊ±ÖÓ
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//	//gpio³õÊ¼»¯½á¹¹Ìå³ÉÔ±¸³Öµ
//	//³õÊ¼»¯PA¡£9£¬¸´ÓÃÍÆÍìÊä³ö
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	//Ö´ÐÐÉÏÃæµÄgpio³õÊ¼»¯
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	// PA.10¸¡¿ÕÊäÈë
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	//USART³õÊ¼»¯
//	//²¨ÌØÂÊÉèÖÃ£¬ÀûÓÃ¿âº¯Êý£¬ÎÒÃÇ¿ÉÒÔÖ±½ÓÕâÑùÅäÖÃ²¨ÌØÂÊ£¬¶ø²»ÐèÒª×ÔÐÐ¼ÆËãUSARTDIVµÄ·ÖÆµÒò×Ó
//	USART_InitStructure.USART_BaudRate = 57600;
//	
//	//ÅäÖÃ´®¿Ú´«ÊäµÄ×Ö³¤¡£±¾Àý³Ì°ÑËüÉèÖÃÎª×î³£ÓÃµÄ8Î»×Ö³¤£¬Ò²¿ÉÒÔÉèÖÃÎª9Î»¡£
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	
//	//ÅäÖÃÍ£Ö¹Î»¡£°ÑÍ¨Ñ¶Ð­ÒéÖÐµÄÍ£Ö¹Î»ÉèÖÃÎª1Î»¡£
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	
//	//ÅäÖÃÆæÅ¼Ð£ÑéÎ»¡£±¾Àý³Ì²»ÉèÖÃÆæÅ¼Ð£ÑéÎ»
//	USART_InitStructure.USART_Parity = USART_Parity_No ;
//	
//	//ÅäÖÃÓ²¼þÁ÷¿ØÖÆ¡£²»²ÉÓÃÓ²¼þÁ÷¡£
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	
//	//ÅäÖÃ´®¿ÚµÄÄ£Ê½¡£ÎªÁËÅäÖÃË«ÏßÈ«Ë«¹¤Í¨Ñ¶£¬ÐèÒª°ÑRxºÍTxÄ£Ê½¶¼¿ªÆô¡£
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//	//Ö´ÐÐÉÏÃæµÄ²Ù×÷
//	USART_Init(USART2, &USART_InitStructure);

//	//×¢Òâ.½ÓÊÕÖÐ¶ÏÊ¹ÄÜ£¬ºÜÖØÒª¡£(¡¦?¡¦£© Ïëµ±³õ£¬¾ÍÊÇÒòÎªÕâ¸ö²½×à£¬µ¢ÎóÁËÒ»ÏÂÎçºÍÒ»ÍíÉÏµÄÊ±¼ä
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//	
//	//µ÷ÓÃUSART_Cmd() Ê¹ÄÜUSART1ÍâÉè¡£ÔÚÊ¹ÓÃÍâÉèÊ±£¬²»½öÒªÊ¹ÄÜÆäÊ±ÖÓ£¬»¹Òªµ÷ÓÃ´Ëº¯ÊýÊ¹ÄÜÍâÉè²Å¿ÉÒÔÕý³£Ê¹ÓÃ¡£
//	USART_Cmd(USART2, ENABLE);
//}
