#ifndef __MPU_SPI_H_
#define __MPU_SPI_H_

#include "stm32f4xx.h"
#include "cmsis_os.h"


void SPI1_Init(void);			 //初始化SPI1口
void SPI2_Init(void);      //初始化SPI2口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);//设置SPI2速度  
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1总线读写一个字节
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2总线读写一个字节
void SPI2_WriteByte(u8 TxData);  //SPI2写一个字节
u8 SPI2_ReadByte(void);          //SPI2读一个字节
void spidmainit(void);
enum {SPICOM1,SPICOM2,SPICOM3};
typedef struct 
{
  SPI_TypeDef        *SPIx;
	SemaphoreHandle_t  WxBinarySemaphore;
	QueueHandle_t      WxMessage_Queue;
	QueueHandle_t      RxMessage_Queue;	
}SPI_DataStruct;





extern SPI_DataStruct   xSPI_Handler[3];
extern xSemaphoreHandle MPUsemaphore;
#endif

