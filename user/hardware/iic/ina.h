#ifndef _INAIIC_H
#define _INAIIC_H


#include "main.h"

// 测量电压不可超过 36v
	

#define INA_IIC_ADDRESS  0X80
#define INA_READ  0X01
#define INA_WRITE  0xfe
///////////  Register Set Summary   ///
// READ and write


typedef struct
{
	u16 voltage;
	int current;
	u16 power;
} InaReal_Data;



//111 连续地转换
//001 010 011 单次转换   转化一次后不许在写入
//省电模式 

// 转换就绪标志位 CVRF 转换完成就位  清除条件 再次设置模式  或 读取 06寄存器
//所有样本都被测量并根据配置寄存器(00h)中设置的平均值取平均值为止。
#define CONFIG_REG  0X00        //0000   给111  001 010 011
#define ENABLE_REG  0X06        //0000
#define ALERT_REG  0X07         //0000

// read

// read
#define CURRENT_REG 0x01          //0X0000
#define VOLTAGE_REG 0X02          //0X0000
#define POWER_REG   0X03           //0X0000



#define  ID_REG    0XFE         //0X5449
#define  DIEID_REG 0XFF         //0X2270

#define  INA_ID   0X5449
#define  DIEID_ID 0X2270

#define  CONFIG_REG_NUM       0x6127
#define  CONFIG_REG_RESET     0x6127|0x8000
#define  CONFIG_REG_SET      0X6000

//Averaging Mode
#define  AVG_NUM_1       0X0<<9
#define  AVG_NUM_4       0X1<<9
#define  AVG_NUM_16      0X2<<9
#define  AVG_NUM_64      0X3<<9
#define  AVG_NUM_128     0X4<<9
#define  AVG_NUM_256     0X5<<9
#define  AVG_NUM_512     0X6<<9
#define  AVG_NUM_1024    0X7<<9
// Bus Voltage Conversion Time
#define  VBUS_TIME_140US      0X0<<6
#define  VBUS_TIME_204US       0X1<<6
#define  VBUS_TIME_332US      0X2<<6
#define  VBUS_TIME_588US      0X3<<6
#define  VBUS_TIME_1MS     0X4<<6
#define  VBUS_TIME_2MS     0X5<<6
#define  VBUS_TIME_4MS     0X6<<6
#define  VBUS_TIME_8MS    0X7<<6
// Bus current Conversion Time
#define  IBUS_TIME_140US      0X0<<3
#define  IBUS_TIME_204US       0X1<<3
#define  IBUS_TIME_332US      0X2<<3
#define  IBUS_TIME_588US      0X3<<3
#define  IBUS_TIME_1MS     0X4<<3
#define  IBUS_TIME_2MS     0X5<<3
#define  IBUS_TIME_4MS     0X6<<3
#define  IBUS_TIME_8MS    0X7<<3
// Bus mode
#define  TRIG_SHUTOWN       0X0
#define  TRIG_CURRENT       0X1
#define  TRIG_VOLTAGE      0X2
#define  TRIG_ALL          0X3
#define  COUT_SHUTDOWN     0X4
#define  CONT_CURRENT     0X5
#define  CONT_VOLTAGE     0X6
#define  CONT_ALL          0X7


extern InaReal_Data INA_Data;

extern 	bool_t INA_int(void);
extern u8 INA260_DataUpdate(void);
#endif

