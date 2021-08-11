#ifndef TIMER_H
#define TIMER_H
#include "main.h"

extern void TIM1_Init(uint16_t arr, uint16_t psc);
extern void TIM3_Init(uint16_t arr, uint16_t psc);
extern void TIM6_Init(uint16_t arr, uint16_t psc);
extern void TIM12_Init(uint16_t arr, uint16_t psc);
extern void PWM_Out_Init_TIM4(uint16_t hz);
#endif
