/************************************************************************/
/*********                      MPU6050.H                       *************/
/**********          Written By LYM---20160916              *************/
/**********                  Version 1.0			      ***************/
/************************************************************************/
#ifndef __BSP_MPU9250_INIT_H__
#define __BSP_MPU9250_INIT_H__
#include "stm32f4xx.h"
#include "delay.h"
#include "stdio.h"	
#include "math.h"
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------宏定义-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================
//注释此宏可关闭打印信息
//#define MPU9250_DUBUG 1
#define SPI_CS1_PIN  GPIO_Pin_3
#define SPI_CS2_PIN  GPIO_Pin_4
#define MPUCS_High  GPIO_SetBits(GPIOC,SPI_CS1_PIN)
#define MPUCS_Low	  GPIO_ResetBits(GPIOC,SPI_CS1_PIN)

#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z


#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高8位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器0x71

#define MPU_ADDR				0x68	//ADO接GND，最低位为0
#define	GYRO_ADDRESS   			0xD0	//陀螺仪地址
#define ACCEL_ADDRESS  			0xD0   	//加速度计地址

#define MAG_ADDRESS   			0x18   	//磁力计地址
#define AKM8963_DEVICE_ID_REG	0x00	//AKM8963器件ID寄存器  0x48
#define AKM8963_INFO_REG		0x01	//AKM8963器件信息寄存器
#define AKM8963_STATUS1_REG		0x02	//AKM8963状态寄存器1

#define AKM8963_MAG_XOUTL_REG	0x03	//AKM8963磁力计值,X轴低8位寄存器
#define AKM8963_MAG_XOUTH_REG	0x04	//AKM8963磁力计值,X轴高8位寄存器
#define AKM8963_MAG_YOUTL_REG	0x05	//AKM8963磁力计值,Y轴低8位寄存器
#define AKM8963_MAG_YOUTH_REG	0x06	//AKM8963磁力计值,Y轴高8位寄存器
#define AKM8963_MAG_ZOUTL_REG	0x07	//AKM8963磁力计值,Z轴低8位寄存器
#define AKM8963_MAG_ZOUTH_REG	0x08	//AKM8963磁力计值,Z轴高8位寄存器

#define AKM8963_STATUS2_REG		0x09	//AKM8963状态寄存器2
#define	AKM8963_CNTL1_REG		0x0A	//AKM8963控制寄存器1
#define	AKM8963_CNTL2_REG		0x0B	//AKM8963控制寄存器2
#define AKM8963_SELF_TEST_REG	0x0C	//AKM8963自检控制寄存器
#define AKM8963_TEST1_REG		0x0D	//AKM8963测试寄存器1
#define AKM8963_TEST2_REG		0x0E	//AKM8963测试寄存器2
#define AKM8963_I2C_DISABLE_REG	0x0F	//AKM8963-I2C失能寄存器
#define AKM8963_ASAX_REG		0x10	//AKM8963磁力计X轴灵敏度校正寄存器
#define AKM8963_ASAY_REG		0x11	//AKM8963磁力计Y轴灵敏度校正寄存器
#define AKM8963_ASAZ_REG		0x12	//AKM8963磁力计Z轴灵敏度校正寄存器

enum _MPU9250_STATUS
{
	MPU9250_OK = 0,
	MPU9250_FAIL = 1,
};
typedef struct __MPU9250_REAL_Data__t
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
    float Mag_X;   //转换成实际的X轴角磁力计，
    float Mag_Y;   //转换成实际的Y轴角磁力计，
    float Mag_Z;   //转换成实际的Z轴角磁力计
}MPU9250_DATA_t;

typedef struct __MPU9250_REAL_Data__
{
    short Accel_X;  //转换成实际的X轴加速度，
    short Accel_Y;  //转换成实际的Y轴加速度，
    short Accel_Z;  //转换成实际的Z轴加速度，
    short Temp;     //转换成实际的温度，单位为摄氏度
    short Gyro_X;   //转换成实际的X轴角加速度，
    short Gyro_Y;   //转换成实际的Y轴角加速度，
    short Gyro_Z;   //转换成实际的Z轴角加速度
    short Mag_X;   //转换成实际的X轴角磁力计，
    short Mag_Y;   //转换成实际的Y轴角磁力计，
    short Mag_Z;   //转换成实际的Z轴角磁力计
	
}MPU9250_DATA;

typedef struct 
{
   short V_X;   
   short V_Y;  
   short V_Z; 
   float Pitch;
	 float ROLL;
	 float YAW;	
	 float Temp;
}Angular_Handle;

//陀螺仪数据结构体
typedef __packed struct 
{
    uint16_t status;
    uint8_t accel[6];
    uint8_t temp[2];
    uint8_t gyro[6];
} mpu6500_real_data_t;

typedef enum _MPU9250_STATUS MPU9250_STATUS;
extern  MPU9250_DATA_t   MPU9250_Real_Data;
extern  float yaw_angle,pitch_angle,roll_angle;
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------外部接口函数-------------------------------------
//=======================================================================
int MPU9250_Init(void);
MPU9250_STATUS MPU9250_Get_Temperature(float *temp);
MPU9250_STATUS MPU9250_Get_Gyroscope(short *gx,short *gy,short *gz);
MPU9250_STATUS MPU9250_Get_Accelerometer(short *ax,short *ay,short *az);
MPU9250_STATUS MPU9250_Get_Mag(short *mx,short *my,short *mz);

//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------内部调用函数-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================
u8 MPU9250_Read_Byte(u8 addr,u8 reg);										//读一个字节
MPU9250_STATUS MPU9250_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);		//连续读数据
MPU9250_STATUS MPU9250_Write_Byte(u8 addr,u8 reg,u8 data);				//写一个字节
MPU9250_STATUS MPU9250_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);	//连续写数据
//-------------------------------------------------------------------------MPU6050 陀螺仪范围设置函数
MPU9250_STATUS MPU9250_Set_Gyro(u8 fsr);	//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//-------------------------------------------------------------------------MPU6050 加速度计范围设置函数
MPU9250_STATUS MPU9250_Set_Accel(u8 fsr); //fsr:0,±2g;1,±4g;2,±8g;3,±16g
//-------------------------------------------------------------------------MPU6050 采样频率设置函数
MPU9250_STATUS MPU9250_Set_Rate(u16 rate);

int MPU6500_Init(void);
#endif
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------具体应用实例-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================

//=======================================================================
//--------------------------End of MPU6050.H---------------------------------
//=======================================================================






