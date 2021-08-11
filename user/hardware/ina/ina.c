#include "ina.h"
#include "delay.h"

void INA_IIC_Configuration(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

}


//void IIC_delay(u16 time)
//{
//	u16 i = 0;
//	while (time--)
//	{
//		i = 10; //自己定义
//		while (i--)
//			;
//	}
//}
///////////////////////////////////////////////////////////////
unsigned char IIC_Start(void)
{
	SDA_OUT(); //sda线输出

	SDA_H;
	SCL_H;
	delay_us(4);
	SDA_L;
	delay_us(4);
	SCL_L;
	return 1;
}

void IIC_Stop(void)
{
	SDA_OUT(); //sda线输出

	SCL_L;
	SDA_L;
	delay_us(4);
	SCL_H;
	SDA_H;
	delay_us(4);
}
//////////////////////////////////////////////////////////////
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}

void IIC_NoAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}

unsigned char IIC_WaitAck(void)
{
	u8 ucErrTime = 0;

	SDA_IN(); //SDA设置为输入
	SDA_H;
	delay_us(1);
	SCL_H;
	delay_us(1);
	while (SDA_read)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;
	return 0;
}
////////////////////////////////////////////////////////////////
void IIC_SendByte(u8 txd)
{
	u8 t;
	SDA_OUT();
	SCL_L; //拉低时钟开始数据传输
	for (t = 0; t < 8; t++)
	{
		if ((txd & 0x80) >> 7)
		{
			SDA_H;
		}
		else
		{
			SDA_L;
		}
		txd <<= 1;
		delay_us(2);
		SCL_H;
		delay_us(2);
		SCL_L;
		delay_us(2);
	}
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_ReadByte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN(); //SDA设置为输入
	for (i = 0; i < 8; i++)
	{
		SCL_L;
		delay_us(2);
		SCL_H;
		receive <<= 1;
		if (SDA_read)
			receive++;
		delay_us(1);
	}
	if (!ack)
		IIC_NoAck(); //发送nACK
	else
		IIC_Ack(); //发送ACK
	return receive;
}
////////////////////////////////////////////////////////////////

#define N 12
InaReal_Data INAReal_Data;

const  InaReal_Data *get_INA_Sensor_Measure_Point(void)
{
 
	return &INAReal_Data;
}

void INA_REG_Write(u8 reg, u16 data)
{
	u8 data_temp[2];
	data_temp[0] = (u8)(data >> 8);
	data_temp[1] = (u8)(data & 0xff);

	IIC_Start();
	IIC_SendByte(INA260_ADDRESS);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_SendByte(data_temp[0]);
	IIC_WaitAck();
	data++;
	IIC_SendByte(data_temp[1]);
	IIC_WaitAck();
	IIC_Stop();
}

void INA_Read_Byte(u8 reg, u8 *data)
{
	IIC_Start();
	IIC_SendByte(INA260_ADDRESS);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();

	IIC_Start();
	IIC_SendByte(INA260_ADDRESS | 0x01);
	IIC_WaitAck();

	*data = IIC_ReadByte(1);
	data++;
	*data = IIC_ReadByte(0);
	IIC_Stop();
}

void INA_Init()
{	
	INA_IIC_Configuration();
	INA_REG_Write(CONFIG_REG, CONFIG_REG_RESET);
	delay_ms(100);
	INA_REG_Write(CONFIG_REG, CONFIG_REG_SET|AVG_NUM_64|VBUS_TIME_1MS|IBUS_TIME_2MS|CONT_ALL);

}


int INA_Get_Current_mA()
{
	u8 data_temp[2];
	int current = 0;
	INA_Read_Byte(CURRENT_REG, data_temp);
	if (data_temp[0] >> 7) //最高位是负值的标志位，为1则进行补码转换
	{
		current = ((!(((data_temp[0] << 8) | data_temp[1]) - 1)) * -1.25);
	}
	else
	{
		current = (int)(((data_temp[0] << 8) | data_temp[1]) * 1.25);
	}
	return current;
}

u16 INA_Get_Voltage_mV()
{
	u8 data_temp[2];
	INA_Read_Byte(VOLTAGE_REG, data_temp);
	return (int)(((data_temp[0] << 8) | data_temp[1]) * 1.25);
}

u16 INA_Get_Power_mW()
{
	u8 data_temp[2];
	INA_Read_Byte(POWER_REG, data_temp);
	return (int)(((data_temp[0] << 8) | data_temp[1]) * 10);
}

u8 INA260_DataUpdate()
{
	INAReal_Data.voltage = INA_Get_Voltage_mV();
	INAReal_Data.current = INA_Get_Current_mA();
	INAReal_Data.power = INA_Get_Power_mW() * 0.1;
	if (INAReal_Data.voltage == 0x0000 || INAReal_Data.voltage == 0xffff)
	{
	}
	else
	{

		return 1;
	}
	return 0;
}
//---------------------------
char INA260_Filter()
{
	char count;

	char value_buf[N];
	int k = 0;
	for (count = 0; count < N; count++)

	{

		value_buf[count] = INA_Get_Current_mA();

		//delay();
	}

	for (int i = 1; i < N; i++)
	{
		if (value_buf[i] > value_buf[k])
			k = i;
	}
	//array[k] 就是最大值
	//k 就是对应下标

	return value_buf[k];
}
