#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "BSP_MPU9250_Init.h"
#include "cmsis_os.h"
#include "mpu_delay.h"
#include "MPU_SPI_Init.h"
#define MPU_IIC   0
short test1, test2;
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------数据变量定义-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================
//-------------------------------------------------------------------------MPU9250初始化函数

void  MPU_SPI_Configuration(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin =  SPI_CS1_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin =  SPI_CS2_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);

    SPI1_Init();
    GPIO_SetBits(GPIOA, SPI_CS1_PIN);
    GPIO_SetBits(GPIOA, SPI_CS2_PIN);

}


MPU9250_STATUS MPU6500_1_init(void)
{
    u8 get_ID = 0;
    get_ID = MPU9250_Read_Byte(1, MPU_DEVICE_ID_REG);
    mpu_delay_ms(10);
    if(get_ID == 0x70)
    {
        MPU9250_Write_Byte(1, MPU_PWR_MGMT1_REG, 0x80); //复位MPU9250
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_PWR_MGMT1_REG, 0x00); //唤醒MPU9250
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_SIGPATH_RST_REG, 0x07); //信号通道复位
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_PWR_MGMT1_REG, 0x03);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_PWR_MGMT2_REG, 0x00);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_CFG_REG, 0x00);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_SAMPLE_RATE_REG, 0x00); //1K采样率
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_CFG_REG, 0x00); //内部低通滤波频率，4000hz
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_GYRO_CFG_REG, 2 << 3);			//陀螺仪传感器,±250dps;1,±500dps;2,±1000dps;3,±2000dps
        mpu_delay_ms(10);

        MPU9250_Write_Byte(1, MPU_ACCEL_CFG_REG, 0 << 3);				//加速度传感器,±2g->0 ±4->1 ±8->2
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_INT_EN_REG, 0x01);		//开启原始数据准备完成中断
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, MPU_INTBP_CFG_REG, 0x01);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, 0x27, 0x0d);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, 0x67, 0x0d);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(1, 0x27, 0x0d);
        mpu_delay_ms(10);

        MPU9250_Write_Byte(1, MPU_PWR_MGMT1_REG, 0x01);
        mpu_delay_ms(10);		//设置CLKSEL,PLL X轴为参考
        MPU9250_Write_Byte(1, MPU_PWR_MGMT2_REG, 0x00);
        mpu_delay_ms(10);	//加速度与陀螺仪都工作



    }
    else return MPU9250_FAIL;



    return MPU9250_OK;
}


MPU9250_STATUS MPU6500_2_init(void)
{
    u8 get_ID = 0;
    get_ID = MPU9250_Read_Byte(2, MPU_DEVICE_ID_REG);
    mpu_delay_ms(10);
    if(get_ID == 0x70)
    {
        MPU9250_Write_Byte(2, MPU_PWR_MGMT1_REG, 0x80); //复位MPU9250
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_PWR_MGMT1_REG, 0x00); //唤醒MPU9250
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_SIGPATH_RST_REG, 0x07); //信号通道复位
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_PWR_MGMT1_REG, 0x03);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_PWR_MGMT2_REG, 0x00);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_CFG_REG, 0x00);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_SAMPLE_RATE_REG, 0x00); //1K采样率
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_CFG_REG, 0x00); //内部低通滤波频率，4000hz
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_GYRO_CFG_REG, 2 << 3);			//陀螺仪传感器,±250dps;1,±500dps;2,±1000dps;3,±2000dps
        mpu_delay_ms(10);

        MPU9250_Write_Byte(2, MPU_ACCEL_CFG_REG, 0 << 3);				//加速度传感器,±2g->0 ±4->1 ±8->2
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_INT_EN_REG, 0x01);		//开启原始数据准备完成中断
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, MPU_INTBP_CFG_REG, 0x01);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, 0x27, 0x0d);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, 0x67, 0x0d);
        mpu_delay_ms(10);
        MPU9250_Write_Byte(2, 0x27, 0x0d);
        mpu_delay_ms(10);

        MPU9250_Write_Byte(2, MPU_PWR_MGMT1_REG, 0x01);
        mpu_delay_ms(10);		//设置CLKSEL,PLL X轴为参考
        MPU9250_Write_Byte(2, MPU_PWR_MGMT2_REG, 0x00);
        mpu_delay_ms(10);	//加速度与陀螺仪都工作



    }
    else return MPU9250_FAIL;



    return MPU9250_OK;
}


int MPU9250_Init(void)
{
    MPU_SPI_Configuration();

    return (MPU6500_1_init() || MPU6500_2_init());

}

int MPU6500_Init(void)
{
    MPU_SPI_Configuration();

    return (MPU6500_1_init() || MPU6500_2_init());

}
//=========================================================================================
//*******************************MPU9250各轴数据********************************************
//=========================================================================================
//-------------------------------------------------------------------------MPU9250 温度函数
MPU9250_STATUS MPU9250_Get_Temperature(float* temp)
{
    u8 buf[2];
    short raw;
    MPU9250_Read_Len(1, MPU_TEMP_OUTH_REG, 2, buf);
//	raw=((u16)(mpu6500_real_data.temp[0])<<8)|mpu6500_real_data.temp[1];
    raw = ((u16)(buf[0]) << 8) | buf[1];
    //temp=36.53+((double)raw)/340;
    *temp = 13 + ((double)raw) / 333.87;
    return MPU9250_OK;
}
//-------------------------------------------------------------------------MPU9250 陀螺仪函数
MPU9250_STATUS MPU9250_Get_Gyroscope(short* gx, short* gy, short* gz)
{
    u8 buf[6];
    short dat[2][3];
    if(MPU9250_Read_Len(1, MPU_GYRO_XOUTH_REG, 6, buf))
    {
        return MPU9250_FAIL;
    }
    else
    {
        dat[0][0] = ((u16)buf[0] << 8) | buf[1];
        dat[0][1] = ((u16)buf[2] << 8) | buf[3];
        dat[0][2] = ((u16)buf[4] << 8) | buf[5];
    }

    if(MPU9250_Read_Len(2, MPU_GYRO_XOUTH_REG, 6, buf))
    {
        return MPU9250_FAIL;
    }
    else
    {
        dat[1][0] = ((u16)buf[0] << 8) | buf[1];
        dat[1][1] = ((u16)buf[2] << 8) | buf[3];
        dat[1][2] = ((u16)buf[4] << 8) | buf[5];
    }

    *gx = (dat[0][0] + dat[1][1]) / 2;
    *gy = (dat[0][1] - dat[1][0]) / 2;
    *gz = (dat[0][2] + dat[1][2]) / 2;

//		*gx=(dat[0][0]+dat[1][1])/2;
//		*gy=(dat[0][1]-0.25*dat[1][0])/2;
//		*gz=(0.707f *dat[0][2]+dat[1][2])/2;

//	  *gx=dat[0][0];
//		*gy=dat[0][1];
//		*gz=dat[0][2];
//	test1 = dat[0][1];
//		*gx=(dat[0][0]+dat[1][1])/2;
//		*gy=(-dat[0][1]+dat[1][0])/2;
//		*gz=(dat[0][2]+dat[1][2])/2;

    return MPU9250_OK;
}
//-------------------------------------------------------------------------MPU9250 加速度计函数
MPU9250_STATUS MPU9250_Get_Accelerometer(short* ax, short* ay, short* az)
{
    u8 buf[6];
    short dat[2][3];
    if(MPU9250_Read_Len(1, MPU_ACCEL_XOUTH_REG, 6, buf))
    {
        return MPU9250_FAIL;
    }
    else
    {
        dat[0][0] = ((u16)buf[0] << 8) | buf[1];
        dat[0][1] = ((u16)buf[2] << 8) | buf[3];
        dat[0][2] = ((u16)buf[4] << 8) | buf[5];
    }

    if(MPU9250_Read_Len(2, MPU_ACCEL_XOUTH_REG, 6, buf))
    {
        return MPU9250_FAIL;
    }
    else
    {
        dat[1][0] = ((u16)buf[0] << 8) | buf[1];
        dat[1][1] = ((u16)buf[2] << 8) | buf[3];
        dat[1][2] = ((u16)buf[4] << 8) | buf[5];
    }

//		*ax=(dat[0][0]+dat[1][0])/2;
//		*ay=(dat[0][1]+dat[1][1])/2;
//		*az=(dat[0][2]+dat[1][2])/2;

    *ax = dat[0][0];
    *ay = dat[0][1];
    *az = dat[0][2];

//		*ax=dat[0][0];
//		*ay=dat[0][1];
//		*az=(dat[0][2]+dat[1][2])/2;

//		*ax=((u16)mpu6500_real_data.accel[0]<<8)|mpu6500_real_data.accel[1];
//		*ay=((u16)mpu6500_real_data.accel[2]<<8)|mpu6500_real_data.accel[3];
//		*az=((u16)mpu6500_real_data.accel[4]<<8)|mpu6500_real_data.accel[5];


//	*ax = mpu6500_real_data.accel[0];
//	*ay = MPU9250_Real_Data.Accel_Y;
//	*az = MPU9250_Real_Data.Accel_Z;
    return MPU9250_OK;
}
//-------------------------------------------------------------------------MPU9250 磁力计函数
MPU9250_STATUS MPU9250_Get_Mag(short* mx, short* my, short* mz)
{
    u8 buf[6];

    if(MPU9250_Read_Byte(MAG_ADDRESS, AKM8963_STATUS1_REG) & 0X01)
    {

        MPU9250_Read_Len(MAG_ADDRESS, AKM8963_MAG_XOUTL_REG, 6, buf);
        MPU9250_Read_Byte(MAG_ADDRESS, AKM8963_STATUS2_REG);
        *mx = ((u16)buf[1] << 8) | buf[0];
        *my = ((u16)buf[3] << 8) | buf[2];
        *mz = ((u16)buf[5] << 8) | buf[4];

        return MPU9250_OK;
    }
    return MPU9250_FAIL;
}

//=========================================================================================
//*******************************MPU9250设置***********************************************
//=========================================================================================
//-------------------------------------------------------------------------MPU9250 陀螺仪范围设置函数
MPU9250_STATUS MPU9250_Set_Gyro(u8 fsr)	//0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
{
    return MPU9250_Write_Byte(MPU_ADDR, MPU_GYRO_CFG_REG, fsr << 3); //根据写函数 返回0成功
}
//-------------------------------------------------------------------------MPU9250 加速度计范围设置函数
MPU9250_STATUS MPU9250_Set_Accel(u8 fsr) //0,±2g;1,±4g;2,±8g;3,±16g
{
    return MPU9250_Write_Byte(MPU_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);
}

//=========================================================================================
//*******************************MPU9250 SPI函数*******************************************
//=========================================================================================
//-------------------------------------------------------------------------MPU9250连续读函数
// addr MPU9250地址
// reg  MPU9250寄存器地址
// len  读取长度
// *buf 数据存储区
MPU9250_STATUS MPU9250_Read_Len(u8 addr, u8 reg, u8 len, u8* buf)
{
    if(addr == 1)
    {
        GPIO_ResetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_ResetBits(GPIOA, SPI_CS2_PIN);
    }

    SPI1_ReadWriteByte(reg | 0x80);
    while(len)
    {
        if(len == 1)//len=1表示只读取一个寄存器
        {
            *buf = SPI1_ReadWriteByte(0XFF);//读取一个字节并回发NACK
        }
        else
        {
            *buf = SPI1_ReadWriteByte(0XFF);//读取一个字节并回发ACK
        }
        buf++;
        reg++;
        len--;
    }

    if(addr == 1)
    {
        GPIO_SetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_SetBits(GPIOA, SPI_CS2_PIN);
    }

    return MPU9250_OK;

}




//-------------------------------------------------------------------------MPU9250连续写函数
MPU9250_STATUS MPU9250_Write_Len(u8 addr, u8 reg, u8 len, u8* buf)
{
    u8 i = 0;

    if(addr == 1)
    {
        GPIO_ResetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_ResetBits(GPIOA, SPI_CS2_PIN);
    }

    SPI1_ReadWriteByte(reg);
    for(i = 0; i < len; i++, reg++)
    {
        SPI1_ReadWriteByte(buf[i]);
    }
    if(addr == 1)
    {
        GPIO_SetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_SetBits(GPIOA, SPI_CS2_PIN);
    }
    return MPU9250_OK;
}

//==========================================================================================
//-------------------------------------------------------------------------MPU9250读一字节函数
u8 MPU9250_Read_Byte(u8 addr, u8 reg)
{
    u8 Return;

    if(addr == 1)
    {
        GPIO_ResetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_ResetBits(GPIOA, SPI_CS2_PIN);
    }

    SPI1_ReadWriteByte(reg | 0x80);
    Return = SPI1_ReadWriteByte(0xff); //读取数据

    if(addr == 1)
    {
        GPIO_SetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_SetBits(GPIOA, SPI_CS2_PIN);
    }
    return Return;
}
//-------------------------------------------------------------------------MPU9250写一字节函数
MPU9250_STATUS MPU9250_Write_Byte(u8 addr, u8 reg, u8 data)
{
    if(addr == 1)
    {
        GPIO_ResetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_ResetBits(GPIOA, SPI_CS2_PIN);
    }

    SPI1_ReadWriteByte(reg);
    SPI1_ReadWriteByte(data); //发送数据

    if(addr == 1)
    {
        GPIO_SetBits(GPIOA, SPI_CS1_PIN);
    }
    else if(addr == 2)
    {
        GPIO_SetBits(GPIOA, SPI_CS2_PIN);
    }

    return MPU9250_OK;
}
