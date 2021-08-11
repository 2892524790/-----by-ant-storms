#ifndef _MPU_TIME_INIT_H_
#define _MPU_TIME_INIT_H_

#include "stm32f4xx.h"    
uint32_t Get_Time_Micros(void);
double Get_Sys_Ticks(void);
float Get_Sys_Ticks_float_ms(void);

void MPU_PWM_TIM2(void);

#endif


