#include "stm32f10x.h"
#include"led.h"
#include"systick.h"
#include "imu_init_position.h"

uint32_t t=1000000;

#define I2C_SCL_0() GPIOB->ODR &= ~(1 << 6)
#define I2C_SCL_1() GPIOB->ODR |= 1 << 6
#define I2C_SDA_0() GPIOB->ODR &= ~(1 << 7)
#define I2C_SDA_1() GPIOB->ODR |= 1 << 7
#define I2C_READ_SDA() GPIOB->IDR & (1 << 7)

#define HMC5883L_ADDRESS 0x3c



void Soft_Delay(uint32_t t)
{
	while(t --);
}

void Soft_I2C_Delay(void)
{
	uint32_t t = 0;			//72MHz,IAR优化级别：low, 
								//t = 10:200 ~ 270KHz
								//t = 3:340 ~ 440KHz;
								//t = 2:350 ~ 480KHz;
								//t = 1:400 ~ 530KHz;
								//t = 0:440 ~ 610KHz;
	Soft_Delay(t);
}

void Soft_I2C_Start(void)
{
	I2C_SDA_1();
	I2C_SCL_1();
	Soft_I2C_Delay();		//1.5us,来源逻辑分析仪
	I2C_SDA_0();
	Soft_I2C_Delay();
	I2C_SDA_0();
	Soft_I2C_Delay();
}

void Soft_I2C_Stop(void)
{

	I2C_SCL_0();			
	Soft_I2C_Delay();
	I2C_SDA_0();
	Soft_I2C_Delay();
	I2C_SCL_1();
	Soft_I2C_Delay();
	I2C_SDA_1();
	Soft_I2C_Delay();
}
void Soft_I2C_Initial(void)
{
	RCC->APB2ENR |= 1 << 3;		//PB
	GPIOB->CRL &= 0x00ffffff;	//PB7 SDA,PB6 SCL
	GPIOB->CRL |= 0x73000000;	//PB7 OD,PB6 PP
	I2C_SDA_1();
	I2C_SCL_1();
}
void Soft_I2C_Write_Byte(uint8_t data)
{
	uint8_t i = 0;
	
	for(i = 0;i < 8;i ++)
	{
		I2C_SCL_0();
		Soft_I2C_Delay();
		
		if(data & 0x80)
			I2C_SDA_1();
		else
			I2C_SDA_0();
		
		data <<= 1;
		Soft_I2C_Delay();
		I2C_SCL_1();
		Soft_I2C_Delay();		
	}
	I2C_SCL_0();
}

uint8_t Soft_I2C_Read_Byte(void)
{
	uint8_t i = 0;
	uint8_t data = 0;
	
	I2C_SDA_1();
	
	for(i = 0;i < 8;i ++)
	{
		data <<= 1;
		
		I2C_SCL_0();
		Soft_I2C_Delay();
		I2C_SCL_1();
		Soft_I2C_Delay();
		
		if(I2C_READ_SDA())
			data |= 0x01;
		else
			;	
	}
	I2C_SCL_0();
	
	return data;
}

void Soft_I2C_Wait_ACK(void)
{

	I2C_SCL_0();
	Soft_I2C_Delay();
	I2C_SDA_1();				//释放SDA
	Soft_I2C_Delay();
	I2C_SCL_1();
	Soft_I2C_Delay();
	if(I2C_READ_SDA() == 0)
	{
		I2C_SCL_0();
		Soft_I2C_Delay();
		return ;
	}	
	else
		;
		
	I2C_SCL_0();
	Soft_I2C_Delay();
}



void Soft_I2C_Send_ACK(uint8_t type)	//0:ack;1:nack
{
	I2C_SCL_0();
	Soft_I2C_Delay();
	if(type == 0)
		I2C_SDA_0();
	else if(type == 1)
		I2C_SDA_1();
	Soft_I2C_Delay();
	
	I2C_SCL_1();
	Soft_I2C_Delay();
	I2C_SCL_0();
	Soft_I2C_Delay();
}

void Soft_I2C_Write_Device_Register(uint8_t device_address,
									uint8_t register_address,
									uint8_t data)
{
	Soft_I2C_Start();
	Soft_I2C_Write_Byte(device_address);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Write_Byte(register_address);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Write_Byte(data);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Stop();
}

void Soft_I2C_Read_Data(uint8_t device_address,
						uint8_t register_address,
						uint8_t *pbuffer,
						uint8_t count)
{
	Soft_I2C_Start();
	Soft_I2C_Write_Byte(device_address);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Write_Byte(register_address);
	Soft_I2C_Wait_ACK();
	
	Soft_I2C_Start();
	Soft_I2C_Write_Byte(device_address + 1);
	Soft_I2C_Wait_ACK();
	
	for(;count >= 2;count --,pbuffer ++)
	{
		*pbuffer = Soft_I2C_Read_Byte();
		Soft_I2C_Send_ACK(0);
	}
	
	*pbuffer = Soft_I2C_Read_Byte();
	Soft_I2C_Send_ACK(1);
	Soft_I2C_Stop();
}
//µ¥×Ö½Ú¶ÁÈ¡*****************************************
unsigned char I2C_ByteRead(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	Soft_I2C_Read_Data(SlaveAddress,REG_Address,&REG_data,1);
	return REG_data;

}		

// ¶¨ÒåMPU9250ÄÚ²¿µØÖ·
//****************************************
#define	SMPLRT_DIV		0x19	//ÍÓÂÝÒÇ²ÉÑùÂÊ£¬µäÐÍÖµ£º0x07(125Hz)
#define	CONFIG			0x1A	//µÍÍ¨ÂË²¨ÆµÂÊ£¬µäÐÍÖµ£º0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//ÍÓÂÝÒÇ×Ô¼ì¼°²âÁ¿·¶Î§£¬µäÐÍÖµ£º0x18(²»×Ô¼ì£¬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//¼ÓËÙ¼Æ×Ô¼ì¡¢²âÁ¿·¶Î§¼°¸ßÍ¨ÂË²¨ÆµÂÊ£¬µäÐÍÖµ£º0x01(²»×Ô¼ì£¬2G£¬5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//µçÔ´¹ÜÀí£¬µäÐÍÖµ£º0x00(Õý³£ÆôÓÃ)
#define	WHO_AM_I		  0x75	//IICµØÖ·¼Ä´æÆ÷(Ä¬ÈÏÊýÖµ0x68£¬Ö»¶Á)


//****************************

#define	GYRO_ADDRESS   0xD0	  //ÍÓÂÝµØÖ·
#define MAG_ADDRESS    0x18   //´Å³¡µØÖ·
#define ACCEL_ADDRESS  0xD0 
uint8_t mpu_read_data[14] = {0,};
short ax = 0,ay = 0,az = 0,gx = 0,gy = 0,gz = 0;	
#define MPU9250_SLAVE_ADDRESS  		0xd0
#define AK8963_SLAVE_ADDRESS (0x0c << 1)
uint8_t ak8963_read_data[7] = {0,};
int16_t mx = 0,my = 0,mz = 0;
	
void MPU9250_Initial(void)
{
	/*
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x6B, 0x00);	    
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x19 , 0x07);	    
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1A , 0x06);	        
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1C , 0x08);	 
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1B, 0x18); 
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x37, 0x02);		//pass through
	*/
	
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	//½â³ýÐÝÃß×´Ì¬
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,CONFIG, 0x06);
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,GYRO_CONFIG, 0x18);//+-2000dps
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,ACCEL_CONFIG, 0x00);//+-16g
}


void AK8963_Initial(void)
{
	/*
	Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x09, 0x18);		
	Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x0a, 0x11);		//连续模式2,100Hz，16bit
	*/
		//MAG
	 Soft_I2C_Write_Device_Register(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
 //  Delayms(5);	
	Soft_Delay(500000);
 Soft_I2C_Write_Device_Register(MAG_ADDRESS,0x09,0x18);
	Soft_Delay(500000);
   Soft_I2C_Write_Device_Register(MAG_ADDRESS,0x0A,0x11);
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
void USART2_Config(void)
{
	//gpio£¬usart³õÊ¼»¯½á¹¹Ìå¶¨Òå
	//µ±gpio¸´ÓÃÊ±£¨¿ªÆôÍâÉèÊ±£©£¬ÐèÒª³õÊ¼»¯Í¬Ê±gpioºÍpppÍâÉè£¬²¢¶ÔËûÃÇÊ¹ÄÜºÍ¿ªÆôÊ±ÖÓ
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//¿ªÆôgpioºÍÍâÉèÊ±ÖÓ
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//gpio³õÊ¼»¯½á¹¹Ìå³ÉÔ±¸³Öµ
	//³õÊ¼»¯PA¡£9£¬¸´ÓÃÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//Ö´ÐÐÉÏÃæµÄgpio³õÊ¼»¯
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PA.10¸¡¿ÕÊäÈë
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART³õÊ¼»¯
	//²¨ÌØÂÊÉèÖÃ£¬ÀûÓÃ¿âº¯Êý£¬ÎÒÃÇ¿ÉÒÔÖ±½ÓÕâÑùÅäÖÃ²¨ÌØÂÊ£¬¶ø²»ÐèÒª×ÔÐÐ¼ÆËãUSARTDIVµÄ·ÖÆµÒò×Ó
	USART_InitStructure.USART_BaudRate = 57600;
	
	//ÅäÖÃ´®¿Ú´«ÊäµÄ×Ö³¤¡£±¾Àý³Ì°ÑËüÉèÖÃÎª×î³£ÓÃµÄ8Î»×Ö³¤£¬Ò²¿ÉÒÔÉèÖÃÎª9Î»¡£
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	//ÅäÖÃÍ£Ö¹Î»¡£°ÑÍ¨Ñ¶Ð­ÒéÖÐµÄÍ£Ö¹Î»ÉèÖÃÎª1Î»¡£
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	
	//ÅäÖÃÆæÅ¼Ð£ÑéÎ»¡£±¾Àý³Ì²»ÉèÖÃÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	
	//ÅäÖÃÓ²¼þÁ÷¿ØÖÆ¡£²»²ÉÓÃÓ²¼þÁ÷¡£
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	//ÅäÖÃ´®¿ÚµÄÄ£Ê½¡£ÎªÁËÅäÖÃË«ÏßÈ«Ë«¹¤Í¨Ñ¶£¬ÐèÒª°ÑRxºÍTxÄ£Ê½¶¼¿ªÆô¡£
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	//Ö´ÐÐÉÏÃæµÄ²Ù×÷
	USART_Init(USART2, &USART_InitStructure);

	//×¢Òâ.½ÓÊÕÖÐ¶ÏÊ¹ÄÜ£¬ºÜÖØÒª¡£(¡¦?¡¦£© Ïëµ±³õ£¬¾ÍÊÇÒòÎªÕâ¸ö²½×à£¬µ¢ÎóÁËÒ»ÏÂÎçºÍÒ»ÍíÉÏµÄÊ±¼ä
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	//µ÷ÓÃUSART_Cmd() Ê¹ÄÜUSART1ÍâÉè¡£ÔÚÊ¹ÓÃÍâÉèÊ±£¬²»½öÒªÊ¹ÄÜÆäÊ±ÖÓ£¬»¹Òªµ÷ÓÃ´Ëº¯ÊýÊ¹ÄÜÍâÉè²Å¿ÉÒÔÕý³£Ê¹ÓÃ¡£
	USART_Cmd(USART2, ENABLE);
}

#define BUFLEN 200

__IO unsigned char Usart2RxBuf[BUFLEN]={0};
__IO unsigned char Usart2RxOkFlag=0;
__IO unsigned int Usart2RxOkIndex=0;


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
unsigned char BUF[10];       //½ÓÊÕÊý¾Ý»º´æÇø
short T_X,T_Y,T_Z,T_T;		 //X,Y,ZÖá£¬ÎÂ¶È
short A_X,A_Y,A_Z,G_X,G_Y,G_Z,M_X,M_Y,M_Z;
short Acc[3],Gyo[3],Mag[3];

void READ_MPU9250(void)
{
	 //¶ÁÈ¡¼ÓËÙ¶È
	 BUF[0]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_XOUT_L); 
   BUF[1]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_XOUT_H);
   A_Y=	(BUF[1]<<8)|BUF[0];
   //A_X/=164; 						   //¶ÁÈ¡¼ÆËãXÖáÊý¾Ý

   BUF[2]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_YOUT_L);
   BUF[3]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_YOUT_H);
   A_X=	(BUF[3]<<8)|BUF[2];
 //  A_Y/=164; 						   //¶ÁÈ¡¼ÆËãYÖáÊý¾Ý
   BUF[4]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_ZOUT_L);
   BUF[5]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_ZOUT_H);
   A_Z=	(BUF[5]<<8)|BUF[4];
 //  A_Z/=164; 			
	
	//¶ÁÈ¡ ÍÓÂÝÒÇ
	 BUF[0]=I2C_ByteRead(GYRO_ADDRESS,GYRO_XOUT_L); 
   BUF[1]=I2C_ByteRead(GYRO_ADDRESS,GYRO_XOUT_H);
   G_Y=	-1*((BUF[1]<<8)|BUF[0]);
  // G_X/=16.4; 						   //¶ÁÈ¡¼ÆËãXÖáÊý¾Ý

   BUF[2]=I2C_ByteRead(GYRO_ADDRESS,GYRO_YOUT_L);
   BUF[3]=I2C_ByteRead(GYRO_ADDRESS,GYRO_YOUT_H);
   G_X=	-1*((BUF[3]<<8)|BUF[2]);
 //  G_Y/=16.4; 						   //¶ÁÈ¡¼ÆËãYÖáÊý¾Ý
   BUF[4]=I2C_ByteRead(GYRO_ADDRESS,GYRO_ZOUT_L);
   BUF[5]=I2C_ByteRead(GYRO_ADDRESS,GYRO_ZOUT_H);
   G_Z=	-1*((BUF[5]<<8)|BUF[4]);
 //  G_Z/=16.4; 	//¶ÁÈ¡¼ÆËãZÖáÊý¾Ý
	 
	 //¶ÁÈ¡´ÅÁ¦¼Æ
	 //Soft_Delay(50000);	
	 Soft_I2C_Write_Device_Register(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
   //Soft_Delay(50000);	
	 Soft_Delay(100000);	
   Soft_I2C_Write_Device_Register(MAG_ADDRESS,0x0A,0x01);
   Soft_Delay(100000);	
	 //Soft_Delay(1000000);
   BUF[0]=I2C_ByteRead (MAG_ADDRESS,MAG_XOUT_L);
   BUF[1]=I2C_ByteRead (MAG_ADDRESS,MAG_XOUT_H);
   M_Y=-1*((BUF[1]<<8)|BUF[0]);

   BUF[2]=I2C_ByteRead(MAG_ADDRESS,MAG_YOUT_L);
   BUF[3]=I2C_ByteRead(MAG_ADDRESS,MAG_YOUT_H);
   M_X=	-1*((BUF[3]<<8)|BUF[2]);
   						   //¶ÁÈ¡¼ÆËãYÖáÊý¾Ý
	 
   BUF[4]=I2C_ByteRead(MAG_ADDRESS,MAG_ZOUT_L);
   BUF[5]=I2C_ByteRead(MAG_ADDRESS,MAG_ZOUT_H);
   M_Z=-1*((BUF[5]<<8)|BUF[4]);
 					       //¶ÁÈ¡¼ÆËãZÖáÊý¾Ý
}
	
float Pitch,Roll,Yaw;
int temp;

float Imu_Init_Position[3];

void Data_Send_Status(float rol,float pit,float yaw);


#define FILTER_LENGTH	10
short A_X_BUF[FILTER_LENGTH];
short A_Y_BUF[FILTER_LENGTH];
short A_Z_BUF[FILTER_LENGTH];
short G_X_BUF[FILTER_LENGTH];
short G_Y_BUF[FILTER_LENGTH];
short G_Z_BUF[FILTER_LENGTH];
short M_X_BUF[FILTER_LENGTH];
short M_Y_BUF[FILTER_LENGTH];
short M_Z_BUF[FILTER_LENGTH];

float ROL_BUF[FILTER_LENGTH];
float PIT_BUF[FILTER_LENGTH];
float YAW_BUF[FILTER_LENGTH];
__IO unsigned int FILTER_CNT=0;

short MAG_MAX[3]={0};
short MAG_MIN[3]={0};
short MAG_OFFSET[3]={0};
int main(void)
{
	int a_x_sum,a_y_sum,a_z_sum,g_x_sum,g_y_sum,g_z_sum,m_x_sum,m_y_sum,m_z_sum,i;
	float rol_sum,pit_sum,yaw_sum;
	Soft_I2C_Initial();
	MPU9250_Initial();
	AK8963_Initial();
	USART1_Config();
	USART_NVIC_Configuration();
	USART2_Config();
//	reset_sensor_fusion();
	
	while(1)
	{
//		Soft_Delay(20000);
//		Soft_Delay(20000);
//		Soft_Delay(20000);
//		
		/*
		Soft_I2C_Read_Data(MPU9250_SLAVE_ADDRESS,0x3B,mpu_read_data,14);

		ax = ((unsigned short)mpu_read_data[0] << 8) | mpu_read_data[1];
		ay = ((unsigned short) mpu_read_data[2] << 8) | mpu_read_data[3];
		az = ((unsigned short)mpu_read_data[4] << 8) | mpu_read_data[5];
		gx = ((unsigned short)mpu_read_data[8] << 8) | mpu_read_data[9];
		gy = ((unsigned short)mpu_read_data[10] << 8) | mpu_read_data[11];
		gz = ((unsigned short)mpu_read_data[12] << 8) | mpu_read_data[13];
	
		Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x37,0x02);//turn on Bypass Mode 
		Soft_Delay(1000);	
		Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x0A,0x01);
		Soft_Delay(50000);	
		
		Soft_I2C_Read_Data(AK8963_SLAVE_ADDRESS,0x03,ak8963_read_data,7);			//连续模式2：100Hz，必须读0x09 ST2！！！读完数据在读ST2作为结束！！！				
		
		mx = ((uint16_t)ak8963_read_data[1] << 8) | ak8963_read_data[0];
		my = ((uint16_t)ak8963_read_data[3] << 8) | ak8963_read_data[2];
		mz = ((uint16_t)ak8963_read_data[5] << 8) | ak8963_read_data[4];
		*/
		
		while(Calibration_Flag==1){
			READ_MPU9250();
			if(M_X>MAG_MAX[0]){
				MAG_MAX[0]=M_X;
			}
			else if(M_X<MAG_MIN[0]){
				MAG_MIN[0]=M_X;
			}
			else{}
				
			if(M_Y>MAG_MAX[1]){
				MAG_MAX[1]=M_Y;
			}
			else if(M_Y<MAG_MIN[1]){
				MAG_MIN[1]=M_Y;
			}
			else{}

			if(M_Z>MAG_MAX[2]){
				MAG_MAX[2]=M_Z;
			}
			else if(M_Z<MAG_MIN[2]){
				MAG_MIN[2]=M_Z;
			}
			else{}				
			
		}
		
		if(Calibration_Ok_Flag==1){
			Calibration_Ok_Flag=0;
			MAG_OFFSET[0]=(MAG_MAX[0]+MAG_MIN[0])/2;
			MAG_OFFSET[1]=(MAG_MAX[1]+MAG_MIN[1])/2;
			MAG_OFFSET[2]=(MAG_MAX[2]+MAG_MIN[2])/2;
		}
		
   
#if 1
		READ_MPU9250();
		
	//	Send_Data(A_X,A_Y,A_Z,G_X,G_Y,G_Z,M_X,M_Y,M_Z);
#else 
 	
		while(Usart2RxOkFlag==0)
		Usart2RxOkFlag=0;
		
		A_Y=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-18)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-17)%BUFLEN];
		A_X=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-16)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-15)%BUFLEN];
		A_Z=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-14)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-13)%BUFLEN];
		G_Y=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-12)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-11)%BUFLEN];
		G_X=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-10)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-9)%BUFLEN];
 		G_Z=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-8)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-7)%BUFLEN];
		M_Y=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-6)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-5)%BUFLEN];
		M_X=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-4)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-3)%BUFLEN];
		M_Z=Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-2)%BUFLEN]<<8|Usart2RxBuf[(Usart2RxOkIndex+BUFLEN-1)%BUFLEN];		
		
#endif	

		
		M_X=M_X-MAG_OFFSET[0];
		M_Y=M_Y-MAG_OFFSET[1];
		M_Z=M_Z-MAG_OFFSET[2];

		
		A_X_BUF[FILTER_CNT]=A_X;
		A_Y_BUF[FILTER_CNT]=A_Y;
		A_Z_BUF[FILTER_CNT]=A_Z;
		G_X_BUF[FILTER_CNT]=G_X;
		G_Y_BUF[FILTER_CNT]=G_Y;
 		G_Z_BUF[FILTER_CNT]=G_Z;
		M_X_BUF[FILTER_CNT]=M_X;
		M_Y_BUF[FILTER_CNT]=M_Y;
		M_Z_BUF[FILTER_CNT]=M_Z;

		a_x_sum=a_y_sum=a_z_sum=g_x_sum=g_y_sum=g_z_sum=m_x_sum=m_y_sum=m_z_sum=0;
		for(i=0;i<FILTER_LENGTH;i++){
			a_x_sum+=A_X_BUF[i];
			a_y_sum+=A_Y_BUF[i];
			a_z_sum+=A_Z_BUF[i];
			g_x_sum+=G_X_BUF[i];
			g_y_sum+=G_Y_BUF[i];
			g_z_sum+=G_Z_BUF[i];
			m_x_sum+=M_X_BUF[i];
			m_y_sum+=M_Y_BUF[i];
			m_z_sum+=M_Z_BUF[i];
		}
		A_X=(a_x_sum/FILTER_LENGTH);
		A_Y=(a_y_sum/FILTER_LENGTH);
		A_Z=(a_z_sum/FILTER_LENGTH);
		G_X=(g_x_sum/FILTER_LENGTH);
		G_Y=(g_y_sum/FILTER_LENGTH);
		G_Z=(g_z_sum/FILTER_LENGTH);
		M_X=m_x_sum/FILTER_LENGTH;
		M_Y=m_y_sum/FILTER_LENGTH;
		M_Z=m_z_sum/FILTER_LENGTH;
		
		
//		A_X=100;A_Y=A_Z=0;
//		M_X=M_Y=M_Z=0;

		
		Send_Data(A_X,A_Y,A_Z,G_X,G_Y,G_Z,M_X,M_Y,M_Z);
		Acc[0]=A_X;Acc[1]=A_Y;Acc[2]=A_Z;
		Gyo[0]=G_X;Gyo[1]=G_Y;Gyo[2]=G_Z;
		Mag[0]=M_X;Mag[1]=M_Y;Mag[2]=M_Z;
		reset_sensor_fusion(Acc,Gyo,Mag,Imu_Init_Position);
		
//		ROL_BUF[FILTER_CNT]=Init_Roll;
//		PIT_BUF[FILTER_CNT]=Init_Pitch;
//		YAW_BUF[FILTER_CNT]=Init_Yaw;
//		rol_sum=pit_sum=yaw_sum=0;
//		for(i=0;i<=FILTER_LENGTH;i++){
//			rol_sum+=ROL_BUF[i];
//			pit_sum+=PIT_BUF[i];
//			yaw_sum+=YAW_BUF[i];
//		}
//		Init_Roll=rol_sum/FILTER_LENGTH;
//		Init_Pitch=pit_sum/FILTER_LENGTH;
//		Init_Yaw=yaw_sum/FILTER_LENGTH;
		
		
		FILTER_CNT++;
		if(FILTER_CNT>=FILTER_LENGTH)
			FILTER_CNT=0;
		
		Data_Send_Status(Imu_Init_Position[0],Imu_Init_Position[1],-1*Imu_Init_Position[2]);
	}
	
}
//int main(void)
//{
//	//my code
//	LED_GPIO_Config();
//	SysTick_Init();
//	while(1)
//	{
//		LED_ON();
//		Delay_us(t);
//		LED_OFF();
//		Delay_us(t);
//	}
//}
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
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

    

