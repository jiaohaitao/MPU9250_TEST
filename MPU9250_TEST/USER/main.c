#include "stm32f10x.h"
#include"led.h"
#include"systick.h"
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



uint8_t mpu_read_data[14] = {0,};
short ax = 0,ay = 0,az = 0,gx = 0,gy = 0,gz = 0;	
#define MPU9250_SLAVE_ADDRESS  		0xd0
#define AK8963_SLAVE_ADDRESS (0x0c << 1)
uint8_t ak8963_read_data[7] = {0,};
int16_t mx = 0,my = 0,mz = 0;
	
void MPU9250_Initial(void)
{
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x6B, 0x00);	    
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x19 , 0x07);	    
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1A , 0x06);	        
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1C , 0x08);	 
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1B, 0x18); 
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x37, 0x02);		//pass through
}


void AK8963_Initial(void)
{

	Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x0a, 0x16);		//连续模式2,100Hz，16bit
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
	
	USART_Cmd(USART1, ENABLE);
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

	



int main(void)
{
	Soft_I2C_Initial();
	MPU9250_Initial();
	AK8963_Initial();
	USART1_Config();
	while(1)
	{
		Soft_Delay(20000);
		Soft_Delay(20000);
		Soft_Delay(20000);
		
		Soft_I2C_Read_Data(MPU9250_SLAVE_ADDRESS,0x3B,mpu_read_data,14);

		ax = ((unsigned short)mpu_read_data[0] << 8) | mpu_read_data[1];
		ay = ((unsigned short) mpu_read_data[2] << 8) | mpu_read_data[3];
		az = ((unsigned short)mpu_read_data[4] << 8) | mpu_read_data[5];
		gx = ((unsigned short)mpu_read_data[8] << 8) | mpu_read_data[9];
		gy = ((unsigned short)mpu_read_data[10] << 8) | mpu_read_data[11];
		gz = ((unsigned short)mpu_read_data[12] << 8) | mpu_read_data[13];
	
		Soft_I2C_Read_Data(AK8963_SLAVE_ADDRESS,0x03,ak8963_read_data,7);			//连续模式2：100Hz，必须读0x09 ST2！！！读完数据在读ST2作为结束！！！				
		
		mx = ((uint16_t)ak8963_read_data[1] << 8) | ak8963_read_data[0];
		my = ((uint16_t)ak8963_read_data[3] << 8) | ak8963_read_data[2];
		mz = ((uint16_t)ak8963_read_data[5] << 8) | ak8963_read_data[4];
		
		Send_Data(ax,ay,az,gx,gy,gz,mx,my,mz);
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
