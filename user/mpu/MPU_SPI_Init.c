#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "MPU_SPI_Init.h"
#include "BSP_MPU9250_Init.h"

SPI_DataStruct   xSPI_Handler[3];
xSemaphoreHandle MPUsemaphore;
#define SPI_INT_PIN GPIO_Pin_4
mpu6500_real_data_t mpu6500_text_data_t;
//以下是SPI模块的初始化代码，配置成主机模式
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    //这里只针对SPI口初始化
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE); //复位SPI1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE); //停止复位SPI1

    SPI_Cmd(SPI1, DISABLE);
//选择模式3(1,1)
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主机模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8位数据模式
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//空闲模式下SCK为1         CPOL控制不传任何数据时电平状态   ,复位CPOL,SCK引脚在空闲状态处低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据采样从第1个时间边沿开始      置0
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS软件管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//波特率
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//大端模式
    SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC多项式
    SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    SPI_Cmd(SPI1, ENABLE); //使能SPI外设

//	SPI1_ReadWriteByte(0xff);//启动传输
}

//SPI3速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB2/2^(SpeedSet+1)
//fAPB1时钟一般为42Mhz
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    SPI1->CR1 &= 0XFFC7;
    SPI1->CR1 |= SPI_BaudRatePrescaler;	//设置SPI2速度
    SPI_Cmd(SPI1, ENABLE);
}

//SPI3 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {}//等待发送区空
    SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个byte  数据
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {} //等待接收完一个byte
    return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
}


