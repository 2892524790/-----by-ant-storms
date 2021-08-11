#ifndef Shoot_Task_H
#define Shoot_Task_H

#include "main.h"
#include "init.h"
#include "pid.h"
#include "Can_Receive.h"

#include "FreeRTOS.h"
#include "task.h"
#include "Ramp_Control.h"
#include "arm_math.h"
#include "stdlib.h"
#include "can.h"
#include "judgement_info.h"
#include "detect_task.h"
#include "user_lib.h"
//�����������ֵ��Χ
#define Half_ecd_range 4096
#define ecd_range 8191
//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
#define ENGLE_DIVER        360.0f/8192.0f
#define ENGLE 						 1.05f

#define FRI_RAMP_RATE				5.5f					//Ħ�����ٶ�б��



typedef enum
{
	shoot_fire    	=  	0,
	shoot_not_fire  = 	1,
	
}shoot_mode_e;

typedef struct
{
  const motor_measure_t *shoot_motor_measure;
  fp32 accel;
  fp32 speed;
	fp32 angle_set;
  fp32 speed_set;
  int16_t give_current;
} Shoot_Motor_t;

typedef struct
{
  const motor_measure_t *dirver_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Dirver_Motor_t;

typedef struct
{
	const RC_ctrl_t *shoot_RC;          //ң����ָ��
	Shoot_Motor_t	motor_friction1;					//Ħ���ֵ������
	Shoot_Motor_t motor_friction2;
	Shoot_Motor_t motor_friction3;
	Dirver_Motor_t motor_dirver;
	Shoot_Motor_t motor_2006;

	shoot_mode_e shoot_mode;              //ģʽ
	pid_type_def friction1_speed_pid;
//	pid_type_def friction1_angle_pid;
	pid_type_def friction2_speed_pid;
//	pid_type_def friction2_angle_pid;

	pid_type_def friction3_speed_pid;
	pid_type_def friction3_angle_pid;

	pid_type_def dirver_speed_pid;

	int16_t   shoot_give_current[3];	
	int16_t   dirver_give_current;
	int16_t   M2006_give_current;

	float     ecd_shoot_engle;
	int remaining_shoot_heat;
	char shoot_flag;
	char heat_flag;

	char block_flag;
}Shoot_t;



void Shoot_task(void *pvParameters);

#endif

