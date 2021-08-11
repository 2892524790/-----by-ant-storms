//#include "BSP_TIME_Init.h"
#include "stm32f4xx.h"
#include "MPU_TIME_Init.h"
#include "cmsis_os.h"

#include "pid.h"
uint32_t Systic = 0;
//TIM1->CH1=PA8

//uint32_t Get_Time_Micros(void)
//{
//	static float last;
//	static float now;
//  last = now;
//	now = Get_Sys_Ticks_float_ms();
//
//
//	return (uint32_t)((now - last)*1000000);
//}
extern pid_type_def pid_imu_tmp;

void MPU_PWM_TIM2(void)
{

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;


    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    //PA2通道3输出使能
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //设置复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//速度2MHZ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);

    /* Compute the prescaler value */
//    PrescalerValue = (uint16_t) ( ( SystemCoreClock/2  ) / hz_set ) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 2000 - 1; //自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 83;//定时器分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性:TIM输出比较极性高


    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);


    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    PID_init(&pid_imu_tmp, PID_POSITION, 2000,  500, 1100, 10, 0);
}

double Get_Sys_Ticks(void)
{
    double sec;
    double value;
    sec = osKernelSysTick();///1000.0;
    value = sec + ((SysTick->LOAD - SysTick->VAL) * 1000 / SysTick->LOAD) * 1e-3;
//	return Systic;
    return value;
}

//得到微秒级的CPU时间
/*
   return float ms time
*/
float Get_Sys_Ticks_float_ms(void)
{
    return (Systic + (TIM7->CNT) / 1000.0);
}


