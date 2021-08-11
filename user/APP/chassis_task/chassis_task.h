#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "start_task.h"
#include "stdlib.h"

#include "task.h"

#include "main.h"
#include "Can_Receive.h"
#include "remote_control.h"
#include "user_lib.h"
#include "IMUTask.h"
#include "gimbal_task.h"
#include "math.h"
#include "arm_math.h"
#include "detect_task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "pid.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 100
/* single 3508 motor maximum speed, unit is rpm */

#define CHASSIS_MAX_SPEED 8000
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED CHASSIS_MAX_SPEED 
#define MAX_CHASSIS_VY_SPEED CHASSIS_MAX_SPEED
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VR_SPEED CHASSIS_MAX_SPEED 

//ң�������֣�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VZ_RC_SEN 0.05f
/* normalized remote controller proportion */
#define RC_RESOLUTION     660.0f
/* remote mode chassis move speed limit */
/* back and forward speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  CHASSIS_MAX_SPEED
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* left and right speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  CHASSIS_MAX_SPEED
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f


//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
/* math relevant */
/* radian coefficient */
#define RADIAN_COEF        57.3f
#define chassis_rotate    0.65
/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* the perimeter of wheel(mm) */
#define PERIMETER              478
/*---------------power-limit-----------------------*/
#define MAX_CHASSIS_LIMIT 40000
#define MAX_CHASSIS_OUT  10000 
#define CHASSIS_IOUT      3000

#define CHASSIS_Kp			 12.0f
#define CHASSIS_Ki			 0.1f
#define CHASSIS_Kd			 0.0f
#define ANGLE_TO_RAD   0.01745329251994329576923690768489f
#define WARNING_REMAIN_POWER 60
#define TIME_STAMP_3MS 1
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 9.5f//9.5
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  	CHASSIS_MAX_SPEED
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 200.0f
//С����PID
#define DODGE_PID_KP	9.5f
#define DODGE_PID_KI	0.0f
#define DODGE_PID_KD	0.0f
#define CHASSIS_DODGE_PID_MAX_OUT					 CHASSIS_MAX_SPEED
#define CHASSIS_DODGE_PID_MAX_IOUT  0.0f


#if (Hero_Robot==0)
/* wheel track distance(mm) */ //�����־�
#define WHEELTRACK             406
/* wheelbase distance(mm) *///ǰ�����
#define WHEELBASE              480

#define power_50_speed  4250
#define power_55_speed  5250
#define power_60_speed  5250
#define power_65_speed  6750
#define power_70_speed  6750
#define power_90_speed  6750
#define power_120_speed 6750

#define vz_50_speed 220
#define vz_55_speed 238
#define vz_60_speed 245//230
#define vz_65_speed 265
#define vz_70_speed 280
#else 

/* wheel track distance(mm) */ //�����־�
#define WHEELTRACK             338.6f
/* wheelbase distance(mm) *///ǰ�����
#define WHEELBASE              374.6f

#define power_50_speed  4200//3900
#define power_55_speed  4500//4250
#define power_60_speed 	4800//4700
#define power_65_speed  5300//5000
#define power_70_speed  6050
#define power_90_speed  6700
#define power_120_speed 6700

#define vz_50_speed 215
#define vz_55_speed 230
#define vz_60_speed 248//248
#define vz_65_speed 268
#define vz_70_speed 260
#endif


#define		chassis_deadline_limit_angle(input, output, dealine ,ref) \
		{																																	\
				if( ((ref-dealine) < ( input ) )&& ((input) < (ref+dealine) ))\
				{																															\
					 output = ref;                                               \
				}                                                              \
				else																														\
				{																																\
					output = input;																								\
				}																																\
		}		
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }		
typedef enum
{
  CHASSIS_RELAX           	= 0,//��������ģʽ
  CHASSIS_INIT           	  = 1,
  CHASSIS_FOLLOW_GIMBAL  	  = 2,//�Զ�������̨
  CHASSIS_SEPARATE_GIMBAL   = 3,//��̨����
  CHASSIS_DODGE_MODE        = 4,//С����
} chassis_mode_e; //��������ģʽ 

typedef struct
{
	const RC_ctrl_t *chassis_rc_ctrl;          //����ʹ�õ�ң����ָ��
	const motor_measure_t *motor_chassis[4];
	const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
	const monitor_t *chassis_monitor_point;
	chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��

	const Angular_Handle *Gimbal_angle_gyro_point;

	pid_type_def chassis_angle_pid;              //���̸���Ƕ�pid
	pid_type_def motor_speed_pid[4];             //���̵���ٶ�pid
	pid_type_def dodge_pid;
	float RC_X_ChassisSpeedRef;			//���Ҷ�̬����
	float RC_Y_ChassisSpeedRef;			//ǰ��̬����
	float RC_Z_ChassisSpeedRef;			//��ת��̬����
	double	vx; // forward/back
  double	vy; // left/right
  double	vz; // 
		
	int16_t	rotate_x_offset;			//����ƫ���Լ�����
  int16_t	rotate_y_offset;

	float	chassis_relative_angle_set;
	
	fp32	wheel_spd_fdb[4];
  fp32	wheel_spd_ref[4];
	fp32	give_current[4];
	float	Chassis_Gyro_Error;//���YAW�����ķ����е�Ƕ�
	
	extKalman_t Error_Kalman;
}chassis_move_t;
extern chassis_move_t chassis_move;

void chassis_task(void *pvParameters);
#endif

