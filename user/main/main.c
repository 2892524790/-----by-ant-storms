
#include "main.h"

/************************************************************/
/************************************************************/
/************************************************************/
//st
#include "start_task.h"

//bsp
#include "SHOOT_TASK.h"
#include "led.h"
#include "delay.h"
#include "remote_control.h"
#include "can.h"
#include "imutask.h"
#include "timer.h"
#include "usart.h"
#include "adc.h"
static void BSP_init(void);

/************************************************************/
/************************************************************/

/************************************************************/
int main()
{

    BSP_init();

    startTask();

    osKernelStart(); //Start the RTOS Kernel
    while(1)
        ;
}
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	
	IWDG_SetPrescaler(prer); //����IWDG��Ƶϵ��

	IWDG_SetReload(rlr);   //����IWDGװ��ֵ

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //ʹ�ܿ��Ź�
}

//ι�������Ź�
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

void BSP_init(void)
{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(SysCoreClock);
    //led_configuration();
    Device_Usart1_ENABLE_Init(115200);     	//��ʼ������
    Device_Usart6_ENABLE_Init(115200);

    remote_control_init();

    CAN_mode_init();
	
#if (Hero_Robot==0)

	#if version_ball == 0
		shoot_red();
	#else
		Adc_Init();
	#endif
#else
	shoot_red();
#endif
	

		red_init();
    LED_Init();
//  IWDG_Init(4,30000);

		delay_ms(500);
    while(MPU6500_Init());
    IMU_Calibration();
    accel_mat_init();
    ACCEL_Calibration();
    MPUHEAT_configuration();

}
