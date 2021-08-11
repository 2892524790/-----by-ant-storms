#ifndef _TX2_TASK_H_
#define	_TX2_TASK_H_
#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "judge_task.h"

#define BLUE 1
#define RED   0
#define ERROR_COLOR 3
//原结构体
typedef __packed struct 
{
	unsigned char head;
	float angle_pitch;
	float angle_yaw;
	float distance;
	unsigned char crc8;
//	unsigned char end;
}PC_Ctrl_t;


//上位机数据转换共用体
typedef union 
{
	PC_Ctrl_t PcDate;
	unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
}PC_Ctrl_Union_t;



typedef struct 
{
	u8 start;//开始
	u8 speed_high;
	u8 speed_low;
	u8 color;//颜色
	u8 pitch[4];
	u8 yaw[4];
	u8 V_Z[2];
	u8 Outpost_State;
	u8 shoot_speed;//射速
	u8 end;//结束
}send_Tx2_t;

typedef struct 
{
//	char head;
	float yaw;
	float pitch;
	unsigned char mode;
	unsigned char color;
	unsigned char speed;
	float yaw_add;
	float pit_add;
	unsigned char CRC8;
}PC_send_measure_t;
extern void Tx2_task(void  *pvParameters);
extern void SEND_PC_TASK(void*  pvParameters);
//extern void SEND_PC_TASK(void  *pvParameters);

extern uint8_t PcDataCheck( uint8_t *pData );
extern const PC_Ctrl_Union_t *get_PC_Ctrl_Measure_Point(void);
void Send_to_PC(USART_TypeDef* USARTx);

#endif /*_TX2_TASK_H_*/



