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

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128
//任务句柄
static TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define IMU_TASK_PRIO		23
//任务堆栈大小	
#define TASK2_STK_SIZE 		1024
//任务句柄
 TaskHandle_t IMUTask_Handler;


//任务优先级
#define Gimbal_TASK_PRIO		18
//任务堆栈大小	
#define Gimbal_STK_SIZE 		512
//任务句柄
static TaskHandle_t GimbalTask_Handler;


//任务优先级
#define CHASSIS_TASK_PRIO		18
//任务堆栈大小	
#define CHASSIS_STK_SIZE 		512
//任务句柄
TaskHandle_t CHASSISHTask_Handler;


//任务优先级
#define SHOOT_TASK_PRIO		17
//任务堆栈大小	
#define SHOOT_STK_SIZE 		512
//任务句柄
static TaskHandle_t SHOOTTask_Handler;


//任务优先级
#define TX2_TASK_PRIO 20
//任务堆栈大小	
#define TX2_TASK_SIZE 256
//任务句柄
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
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建任务
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
		xTaskCreate((TaskFunction_t )UI_task, 			//任务函数
								(const char* )"UI_task", 			//任务名称
								(uint16_t )UI_TASK_SIZE, 				//任务堆栈大小
								(void* )NULL, 							//传递给任务函数的参数
								(UBaseType_t )UI_TASK_PRIO, 			//任务优先级
								(TaskHandle_t* )&UI_TASK_Handler); 	//任务句柄
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}
void startTask(void)   
{
		//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度								
}


