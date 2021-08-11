/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "arm_math.h"
#include "math.h"
#include "cmsis_os.h"
#include "delay.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
//typedef float fp32;
typedef double fp64;

//��̨�������can����ʧ�ܵ����������ʹ�� ����ӳٷ��Ϳ���ָ��ķ�ʽ���
#define GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE 0

#define SysCoreClock 168

#define version_ball 0//0Ϊ�� , 1Ϊ��(�ӵ�)
#define Hero_Robot 1//0Ϊ��Ӣ�� 1Ϊ��Ӣ��



#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5

#define JUDGE_NVIC 6 
#define PC_NVIC 7
#define CAN1_NVIC 2
#define CAN2_NVIC 1
#define TIM3_NVIC 5
#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

//.......
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) //0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) //0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) //0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) //0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) //0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) //0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) //0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) //0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) //0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) //0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) //0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) //0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) //0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) //0x40011E08

//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //���
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //����

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //���
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //����

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //���
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //����

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //���
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //����

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //���
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //����

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //���
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //����

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //���
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //����

#define VAL_LIMIT(val, min, max) \
  do                             \
  {                              \
    if ((val) <= (min))          \
    {                            \
      (val) = (min);             \
    }                            \
    else if ((val) >= (max))     \
    {                            \
      (val) = (max);             \
    }                            \
  } while (0)

 #define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
	void IWDG_Feed(void);

#endif /* __MAIN_H */
