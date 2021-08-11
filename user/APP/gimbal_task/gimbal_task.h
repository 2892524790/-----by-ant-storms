#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "can_receive.h"
#include "can.h"
#include "FreeRTOS.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "task.h"
#include "pid.h"
#include "IMUTask.h"
#include "Tx2_task.h"
#include "detect_task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "Tx2_task.h"
#include "main.h"

//************************************************
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 200

/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE    (8192.0f/360.0f)
/* the deceleration ratio of pitch axis motor */
#define PIT_DECELE_RATIO       1.0f
/* the deceleration ratio of yaw axis motor *//*偏航轴电机的减速比*/
#define YAW_DECELE_RATIO       1.0f
/* the positive direction of pitch axis motor */
#define PIT_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of yaw axis motor *//*偏航轴电机正方向*/
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of tirgger motor *//*电机正方向*/
#define TRI_MOTO_POSITIVE_DIR  1.0f

//yaw,pitch控制通道以及状态开关通道
#define YawChannel 		2
#define PitchChannel 	3
//remote control parameters
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.0002f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.0005f
#define kKey_Gyro_Pitch 0.0015f
#define kKey_Gyro_Yaw 0.0030f
//S1 S2 拨杆
#define ModeChannel_R 	0
#define ModeChannel_L 	1

#if Hero_Robot==0
//初始化
#define yaw_init_angle_kp 5
#define yaw_init_speed_kp 350

#define pitch_init_angle_kp 15
#define pitch_init_speed_kp 200

//跟随
#define yaw_follow_angle_kp 38
#define yaw_follow_speed_kp 225

#define pitch_follow_angle_kp 21
#define pitch_follow_angle_ki 0.0f
#define pitch_follow_speed_kp 500
//自瞄
#define yaw_track_angle_kp 22
#define yaw_track_angle_ki 0.1f
#define yaw_track_speed_kp 160

#define pitch_track_angle_kp 21
#define pitch_track_angle_ki 0.1f
#define pitch_track_speed_kp 490
//归中设置
#define Glimbal_Yaw_Offset  4249
#define Glimbal_Pitch_Offset 6054
//#define Glimbal_Yaw_Offset_Left  6047
#define Glimbal_Yaw_Offset_Back  21
//#define Glimbal_Yaw_Offset_Right 2019

//俯仰角限幅 单位 度
#define PITCH_MIN  -31 //抬头
#define PITCH_MAX   15 //低头

#else
//新英雄
//初始化
#define yaw_init_angle_kp 5
#define yaw_init_speed_kp 350

#define pitch_init_angle_kp 15
#define pitch_init_speed_kp 150

//跟随
#define yaw_follow_angle_kp 25
#define yaw_follow_speed_kp 225

#define pitch_follow_angle_kp 21
#define pitch_follow_angle_ki 0.0f
#define pitch_follow_speed_kp	440
//自瞄
#define yaw_track_angle_kp 28
#define yaw_track_angle_ki 0.03f
#define yaw_track_speed_kp 328
//**********************************************//
#define pitch_track_angle_kp 30
#define pitch_track_angle_ki 0.05f
#define pitch_track_angle_kd 0
#define pitch_track_speed_kp 250
//
#define Glimbal_Yaw_Offset  1585
#define Glimbal_Pitch_Offset 2116
//#define Glimbal_Yaw_Offset_Left  3637
#define Glimbal_Yaw_Offset_Back  5609
//#define Glimbal_Yaw_Offset_Right 7782
//俯仰角限幅 单位 度
#define PITCH_MIN		-17 //抬头
#define PITCH_MAX   26 //低头

#endif

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
#define LimitMax1(input, max0_out)   \
{                                \
		if (input > max0_out)       \
		{                            \
				input = max0_out;       \
		}                            \
		else if (input < -max0_out) \
		{                            \
				input = -max0_out;      \
		}                            \
}
typedef enum
{
  GIMBAL_RELAX         = 0,//云台无力 0
  GIMBAL_INIT          = 1,//云台初始化(归中) 1
  GIMBAL_FOLLOW_MODE   =2,//云台手动控制 2
	GIMBAL_DODGE_MODE			=3,
	GIMBAL_TRACK_ARMOR		=4,
} gimbal_motor_mode_e;

typedef struct
{
	fp32 kp;
	fp32 ki;
	fp32 kd;

	fp32 set;
	fp32 get;
	fp32 max_out;
	fp32 max_iout;

	fp32 error[3];
	fp32 Dbuf[3];

	fp32 Pout;
	fp32 Iout;
	fp32 Dout;

	fp32 out;
} Gimbal_PID_t;

typedef struct
{  
	const motor_measure_t *gimbal_motor_measure;//电机反馈
	pid_type_def gimbal_motor_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
	uint16_t offset_ecd;
	fp32 relative_angle; 
	fp32 absolute_angle;  	
	fp32 motor_gyro;
	fp32 motor_gyro_set;
	fp32 gimbal_angle_set;
  fp32 current_set;       
  int16_t given_current;  //这个值直接传给电机
	extKalman_t Error_Kalman;
	fp32 pc;
} Gimbal_Motor_t;

typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;
	const Angular_Handle *gimbal_angle_gyro_point;
	const PC_Ctrl_Union_t *PC_Ctrl_Measure_Point;
	const monitor_t *gimbal_monitor_point;
	
	Gimbal_Motor_t 		gimbal_yaw_motor;
	Gimbal_Motor_t		gimbal_pitch_motor;
	gimbal_motor_mode_e gimbal_mode;
	gimbal_motor_mode_e last_gimbal_mode;

	float pitch_angle_dynamic_ref;			//角度动态输入
  float yaw_angle_dynamic_ref;
	
	int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
	fp32		 			pitch_angle_PcCtrl_ref;
	fp32					yaw_angle_PcCtrl_ref;
	int32_t       pit_current;
	int32_t       yaw_current;  
}Gimbal_Control_t;

void gimbal_task(void *pvParameters);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
int16_t get_relative_angle(int16_t raw_ecd, int16_t center_offset);
extern int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);
extern bool_t gimbal_cmd_to_shoot_stop(void);

#endif
