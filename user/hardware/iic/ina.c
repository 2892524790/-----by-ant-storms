#include "ina.h"
#include "iic.h"

InaReal_Data INA_Data;

bool_t INA_REG_Write(u8 reg,u16 data)
{
  u8 data_temp[2];
	data_temp[0] = (u8)(data >> 8);
	data_temp[1] = (u8)(data & 0xff);
 IIC_Start();
 IIC_Send_Byte(INA_IIC_ADDRESS&INA_WRITE);
if (IIC_Wait_Ack()==1)
{
	return 0;
}

IIC_Send_Byte(reg);

if (IIC_Wait_Ack()==1)
{
	return 0;
}
IIC_Send_Byte(data_temp[0]);

if (IIC_Wait_Ack()==1)
{
	return 0;
}
IIC_Send_Byte(data_temp[1]);

if (IIC_Wait_Ack()==1)
{
	return 0;
}

IIC_Stop();

return 1;
}

bool_t INA_REG_Read(u8 reg ,u8 *data )
{
  IIC_Start();
	IIC_Send_Byte(INA_IIC_ADDRESS|INA_READ);
	if (IIC_Wait_Ack()==1)
{
	return 0;
}

  *data = IIC_Read_Byte(1);
  data++;
	*data = IIC_Read_Byte(0);
	IIC_Stop();
 
return 1;
}


int INA_Read(u8 reg)
{
	 u8 data[2];
	
	  INA_REG_Read(reg,data);
	
   return (data[0]<<8|data[1]);
}


bool_t INA_int(void)
{
	IIC_Init();
	delay_ms(500);
  if( INA_REG_Write(CONFIG_REG , 0x6527) == 0 ) return 0;
	delay_ms(500);

	
	// 自检
	if(INA_Read(CONFIG_REG) == 0) return 0;
	if(INA_Read(ID_REG) == 0  ) return 0;
	if(INA_Read(DIEID_REG) == 0) return 0;
	
	delay_ms(500);
	
	//初始化
   if(INA_REG_Write(CONFIG_REG,(u16)(0x6527)) == 0) 
		 
	 return 0;
	 
	 return 1;
	 
}

int INA_Get_Current_mA()
{
	u8 data_temp[2];
	int current = 0;
	INA_REG_Read(CURRENT_REG, data_temp);
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
	INA_REG_Read(VOLTAGE_REG, data_temp);
	return (int)(((data_temp[0] << 8) | data_temp[1]) * 1.25);
}

u16 INA_Get_Power_mW()
{
	u8 data_temp[2];
	INA_REG_Read(POWER_REG, data_temp);
	return (int)(((data_temp[0] << 8) | data_temp[1]) * 10);
}

u8 INA260_DataUpdate()
{
	INA_Data.voltage = INA_Get_Voltage_mV();
	INA_Data.current = INA_Get_Current_mA();
	INA_Data.power = INA_Get_Power_mW() ;
	if (INA_Data.voltage == 0x0000 || INA_Data.voltage == 0xffff)
	{
		
	}
	else
	{
		return 1;
	}
	return 0;
}



