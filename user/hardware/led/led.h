#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���

#define LED1_PIN                  GPIO_Pin_10
#define LED1_GPIO_PORT            GPIOC
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOC


#define LED2_PIN                  GPIO_Pin_11
#define LED2_GPIO_PORT            GPIOC
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define LED3_PIN                  GPIO_Pin_12
#define LED3_GPIO_PORT            GPIOC
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define LED4_PIN                  GPIO_Pin_2
#define LED4_GPIO_PORT            GPIOD
#define LED4_GPIO_CLK             RCC_AHB1Periph_GPIOD



/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  0
#define OFF 1

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN)


#define LED4(a)	if (a)	\
					GPIO_SetBits(LED4_GPIO_PORT,LED4_PIN);\
					else		\
					GPIO_ResetBits(LED4_GPIO_PORT,LED4_PIN)
					
					
#define LED1_TOGGLE()			GPIO_ToggleBits(LED1_GPIO_PORT,LED1_PIN)
#define LED2_TOGGLE()			GPIO_ToggleBits(LED2_GPIO_PORT,LED2_PIN)
#define LED3_TOGGLE()			GPIO_ToggleBits(LED3_GPIO_PORT,LED3_PIN)
#define LED4_TOGGLE()			GPIO_ToggleBits(LED4_GPIO_PORT,LED4_PIN)

#define LED_ALLOFF	\
					LED1(OFF);\
					LED2(OFF);\
					LED3(OFF);\
                    LED4(OFF)

#define LED_ALLTOGGLE	\
					LED1_TOGGLE();\
					LED2_TOGGLE();\
					LED3_TOGGLE();\
                    LED4_TOGGLE()

#define LED_ALLON	\
					LED1(ON);\
					LED2(ON);\
					LED3(ON);\
                    LED4(ON)
					
// ��ˮ��
#define DETECT_FLOW_LED_ON(num)	 		GPIO_ResetBits(GPIOC, GPIO_Pin_12 >> (num));
#define DETECT_FLOW_LED_OFF(num)	 	GPIO_SetBits(GPIOC, GPIO_Pin_12 >> (num));
void LED_Init(void);//��ʼ��		
void shoot_red(void);

void red_init(void);
extern void MPUHEAT_configuration(void);
extern void MPUHEAT_on(void);
extern void MPUHEAT_off(void);
#endif

