#include "start_task.h"
#include "main.h"

#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "sys.h"
#include "delay.h"

#include "can.h"
#include "can_receive.h"
#include "rc.h"
#include "led.h" 

#include "remote_control.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

//#include "iwdg.h"
#include "PID.h"
#include "user_lib.h"
#include "Judge_Task.h"
#include "IMUTask.h"
#include "gimbal_task.h" 
#include "CHASSIS_TASK.h"
#include "shoot_task.h"
#include "TX2_task.h"
#include "detect_task.h"
#include "judgement_info.h"

#include "RM_Client_UI.h"

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128
//������
static TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define IMU_TASK_PRIO		23
//�����ջ��С	
#define TASK2_STK_SIZE 		1024
//������
 TaskHandle_t IMUTask_Handler;


//�������ȼ�
#define Gimbal_TASK_PRIO		18
//�����ջ��С	
#define Gimbal_STK_SIZE 		512
//������
static TaskHandle_t GimbalTask_Handler;


//�������ȼ�
#define CHASSIS_TASK_PRIO		18
//�����ջ��С	
#define CHASSIS_STK_SIZE 		512
//������
TaskHandle_t CHASSISHTask_Handler;


//�������ȼ�
#define SHOOT_TASK_PRIO		17
//�����ջ��С	
#define SHOOT_STK_SIZE 		512
//������
static TaskHandle_t SHOOTTask_Handler;


//�������ȼ�
#define TX2_TASK_PRIO 20
//�����ջ��С	
#define TX2_TASK_SIZE 256
//������
static TaskHandle_t TX2_TASK_Handler;

#define JUDGE_TASK_PRIO 19
#define JUDGE_TASK_SIZE 256
static TaskHandle_t JUDGE_TASK_Handler;

//#define IWDG_TASK_PRIO 6
//#define IWDG_TASK_SIZE 128
//static TaskHandle_t IWDG_TASK_Handler;

#define SEND_TASK_PRIO 20
#define SEND_TASK_SIZE 64
static TaskHandle_t SEND_TASK_Handler;

#define DETECT_TASK_PRIO 10
#define DETECT_TASK_SIZE 128
static TaskHandle_t DETECT_TASK_Handler;

#define UI_TASK_PRIO 16
#define UI_TASK_SIZE 256
static TaskHandle_t UI_TASK_Handler;
//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //��������
		xTaskCreate((TaskFunction_t )IMU_task, 
								(const char*    )"IMU_task", 
								(uint16_t       )512, 
								(void*          )NULL, 
								(UBaseType_t    )IMU_TASK_PRIO,
								(TaskHandle_t*  )&IMUTask_Handler);				
    xTaskCreate((TaskFunction_t )gimbal_task,             
                (const char*    )"gimbal_task",           
                (uint16_t       )Gimbal_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )Gimbal_TASK_PRIO,        
                (TaskHandle_t*  )&GimbalTask_Handler);   
	  xTaskCreate((TaskFunction_t )chassis_task,     	
                (const char*    )"chassis_task",   	
                (uint16_t       )CHASSIS_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )CHASSIS_TASK_PRIO,	
                (TaskHandle_t*  )&CHASSISHTask_Handler);   
    xTaskCreate((TaskFunction_t )Shoot_task,             
                (const char*    )"shoot_task",           
                (uint16_t       )SHOOT_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )SHOOT_TASK_PRIO,        
                (TaskHandle_t*  )&SHOOTTask_Handler);
		xTaskCreate((TaskFunction_t )Tx2_task, 
								(const char*    )"Tx2_task", 
								(uint16_t       )TX2_TASK_SIZE, 
								(void*          )NULL, 
								(UBaseType_t    )TX2_TASK_PRIO,  
								(TaskHandle_t*  )&TX2_TASK_Handler);
		xTaskCreate((TaskFunction_t )Judge_task, 
								(const char*    )"Judge_task", 
								(uint16_t       )JUDGE_TASK_SIZE, 
								(void*          )NULL, 
								(UBaseType_t    )JUDGE_TASK_PRIO,  
								(TaskHandle_t*  )&JUDGE_TASK_Handler);		
		xTaskCreate((TaskFunction_t )detect_task, 
								(const char*    )"detect_task", 
								(uint16_t       )DETECT_TASK_SIZE, 
								(void*          )NULL, 
								(UBaseType_t    )DETECT_TASK_PRIO,  
								(TaskHandle_t*  )&DETECT_TASK_Handler);
//	xTaskCreate((TaskFunction_t )iwdg_task, 
//							(const char*    )"iwdg_task", 
//							(uint16_t       )IWDG_TASK_SIZE, 
//							(void*          )NULL, 
//							(UBaseType_t    )IWDG_TASK_PRIO,  
//							(TaskHandle_t*  )&IWDG_TASK_Handler);
		xTaskCreate((TaskFunction_t )SEND_PC_TASK, 
							(const char*    )"SEND_PC_TASK", 
							(uint16_t       )SEND_TASK_SIZE, 
							(void*          )NULL, 
							(UBaseType_t    )SEND_TASK_PRIO,  
							(TaskHandle_t*  )&SEND_TASK_Handler);
		xTaskCreate((TaskFunction_t )UI_task, 			//������
								(const char* )"UI_task", 			//��������
								(uint16_t )UI_TASK_SIZE, 				//�����ջ��С
								(void* )NULL, 							//���ݸ��������Ĳ���
								(UBaseType_t )UI_TASK_PRIO, 			//�������ȼ�
								(TaskHandle_t* )&UI_TASK_Handler); 	//������
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
void startTask(void)   
{
		//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������								
}


