#include "CHASSIS_TASK.h"
#include "gimbal_task.h"
#include "math.h"
#include "judgement_info.h"

//防止遥控处于中间数值却不为零
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
//底盘运动数据
chassis_move_t chassis_move;
extern Gimbal_Control_t gimbal_control;
extern ext_power_heat_data_t	PowerHeatData;				//0x0202
extern ext_game_robot_state_t	GameRobotStat;				//0x0201
u8 key_q=0,flag_q=0;
u8 key_g=0,flag_g=0;

//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘分解
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//初始化
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_give_current(chassis_move_t *chassis_current);
static void chassis_Behaviour_update(chassis_move_t *chassis_behaviour_update);
static void mecanum_calc(float vx, float vy, float vz, float wheel_rpm[]);
static void Chassis_Power_Limit(chassis_move_t *chassis_limit);		
static void Super_power_ctrl(chassis_move_t *power_ctrl);

		
int16_t   timeXFron,    timeXBack,    timeYLeft,    timeYRigh;//键盘  s  w  d   a
//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back; 
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
uint8_t Shift_Flag = 2, Ctrl_Flag=0;
 int F_Flag=0,FFlag_state=0;
int  set_sprm;
void chassis_task(void *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);
		for(;;)
	{
	 chassis_set_mode(&chassis_move);
   chassis_feedback_update(&chassis_move);
   chassis_Behaviour_update(&chassis_move);
   Chassis_Power_Limit(&chassis_move);
	 Super_power_ctrl(&chassis_move);

#if detect
		if (!(toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE)))
	 {
		if(chassis_move.chassis_mode == CHASSIS_RELAX || toe_is_error(DBUSTOE))
		{
			CAN_CMD_CHASSIS(0, 0, 0, 0);
		}
		else
		{
				 CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
						chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		}
	 }
#else 
	 CAN_CMD_CHASSIS(chassis_move.give_current[0], chassis_move.give_current[1],
					chassis_move.give_current[2], chassis_move.give_current[3]);

#endif
		vTaskDelay(5);
	}

}
static int i;
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
    {
        return;
    }
		//底盘开机状态为停止
		for(i = 0; i < 4; i++)
		{	
			chassis_move.give_current[i]=0;
		}
		//获取遥控器指针
	chassis_move_init->chassis_rc_ctrl = get_remote_control_point();
		//陀螺仪姿态指针
	chassis_move_init->Gimbal_angle_gyro_point=get_Gyro_Angle_Point();
			//获取检测系统数据指针
	chassis_move_init->chassis_monitor_point = getErrorListPoint();

    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i]=get_Chassis_Motor_Measure_Point(i);
		PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, MAX_CHASSIS_OUT, CHASSIS_IOUT,CHASSIS_Kp,CHASSIS_Ki,CHASSIS_Kd);
	}
	//*********************************************************************************
   //初始化旋转PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION,CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,CHASSIS_FOLLOW_GIMBAL_PID_KP,CHASSIS_FOLLOW_GIMBAL_PID_KI,CHASSIS_FOLLOW_GIMBAL_PID_KD);
	
	PID_init(&chassis_move_init->dodge_pid,PID_POSITION,CHASSIS_DODGE_PID_MAX_OUT,CHASSIS_DODGE_PID_MAX_IOUT,DODGE_PID_KP,DODGE_PID_KI,DODGE_PID_KD);

}


/****************遥控的模式切换*****************/
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
		chassis_move_mode->last_chassis_mode=chassis_move_mode->chassis_mode;
		
			//遥控器设置行为模式   根据S选择模式控制底盘的行为
    if(switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[1])&&switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[0]))
		{
			if(IF_KEY_PRESSED_F==0)
			{
				F_Flag = 1;
			}
			
			if (IF_KEY_PRESSED_F&& F_Flag == 1)
			{
				F_Flag = 0;
				FFlag_state ++;
				FFlag_state %= 2;
			}
			if(FFlag_state == 0) 
			{
				chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
			}
			else if(FFlag_state){chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE; }	
			
			if((!(chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT))!= 0) Shift_Flag=2;
			if ((chassis_move_mode->chassis_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)&&(Super_power.input_volt > 17000))
				{Shift_Flag = 1;}
			else 
				{Shift_Flag = 2;}
			
			if(IF_KEY_PRESSED_CTRL)	Ctrl_Flag=1;
			else Ctrl_Flag=0;
		}
		else
		{
			flag_g=flag_q=FFlag_state=0;
			if (switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[1])&&switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[0]))   //左下右下  无力
			{
					chassis_move_mode->chassis_mode = CHASSIS_RELAX ;
			}
			else if (switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[1])&&switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[0]))  //左下右中 初始化
			{
					chassis_move_mode->chassis_mode = CHASSIS_INIT;
			}
			else if(!switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[1])&&switch_is_mid(chassis_move_mode->chassis_rc_ctrl->rc.s[0]))
			{
				 chassis_move_mode->chassis_mode =	CHASSIS_FOLLOW_GIMBAL;//当左中，跟随控制
			}
			else if(!switch_is_down(chassis_move_mode->chassis_rc_ctrl->rc.s[1])&&switch_is_up(chassis_move_mode->chassis_rc_ctrl->rc.s[0]))
			{
					chassis_move_mode->chassis_mode = CHASSIS_DODGE_MODE;
			}
			else 
					chassis_move_mode->chassis_mode = CHASSIS_RELAX ;
		}
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	static portTickType  ulCurrentTime = 0;
	static uint32_t  ulDelay = 0;				
	if (chassis_move_update == NULL)
    {
        return;
    }

	chassis_move_update->wheel_spd_fdb[0]=chassis_move_update->motor_chassis[0]->speed_rpm;
	chassis_move_update->wheel_spd_fdb[1]=chassis_move_update->motor_chassis[1]->speed_rpm;
	chassis_move_update->wheel_spd_fdb[2]=chassis_move_update->motor_chassis[2]->speed_rpm;
	chassis_move_update->wheel_spd_fdb[3]=chassis_move_update->motor_chassis[3]->speed_rpm;
		//遥控器数据更新
	if(switch_is_down(chassis_move_update->chassis_rc_ctrl->rc.s[1])&&switch_is_up(chassis_move_update->chassis_rc_ctrl->rc.s[0]))
	{
		ulCurrentTime = xTaskGetTickCount();//当前系统时间
		if (ulCurrentTime >= ulDelay)//每3ms变化一次斜坡量
		{
			ulDelay = ulCurrentTime + TIME_STAMP_3MS;
			if (IF_KEY_PRESSED_W)
				timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
			if (IF_KEY_PRESSED_S)
				timeXFron = 0;//同理
			if (IF_KEY_PRESSED_D)
				timeYLeft = 0;
			if (IF_KEY_PRESSED_A)
				timeYRigh = 0;
			Slope_Chassis_Move_Fron = (int16_t)( CHASSIS_RC_MAX_SPEED_X * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_W, &timeXFron, 1, 100 ) );

			Slope_Chassis_Move_Back = (int16_t)( -CHASSIS_RC_MAX_SPEED_X * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_S, &timeXBack, 1, 100) );

			Slope_Chassis_Move_Left = (int16_t)( -CHASSIS_RC_MAX_SPEED_Y * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_A, &timeYLeft, 1, 100 ) );

			Slope_Chassis_Move_Righ = (int16_t)( CHASSIS_RC_MAX_SPEED_Y * 
					Chassis_Key_MoveRamp( IF_KEY_PRESSED_D, &timeYRigh, 1, 100 ) );
			chassis_move_update->RC_Y_ChassisSpeedRef = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//前后计算
			chassis_move_update->RC_X_ChassisSpeedRef = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;//左右计算
		}
	}
	else
	{
		chassis_move_update->RC_X_ChassisSpeedRef =chassis_move_update ->chassis_rc_ctrl->rc.ch[0]/RC_RESOLUTION*CHASSIS_RC_MAX_SPEED_X;
		chassis_move_update->RC_Y_ChassisSpeedRef = chassis_move_update->chassis_rc_ctrl->rc.ch[1]/RC_RESOLUTION*CHASSIS_RC_MAX_SPEED_Y;
		chassis_move_update->RC_Z_ChassisSpeedRef = -chassis_move_update->chassis_rc_ctrl->rc.ch[2]*CHASSIS_VZ_RC_SEN*10;
	}
}

static void chassis_Behaviour_update(chassis_move_t *chassis_behaviour_update)
{
	static fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	static fp32 relative,Rotation_rate = 0.0f;
	
		if(IF_KEY_PRESSED_Q && key_q == 0)
			{
					key_q = 1;
					flag_q = !flag_q;
			}
			else if(!IF_KEY_PRESSED_Q)
					key_q = 0;
	
		if(IF_KEY_PRESSED_G && key_g == 0&&FFlag_state==0)
			{
					key_g = 1;
					flag_g = !flag_g;
			}
			else if(!IF_KEY_PRESSED_G)
					key_g = 0;
			
	if (chassis_behaviour_update == NULL)
    {
        return;
    }
		
	 if(chassis_behaviour_update->chassis_mode==CHASSIS_RELAX)
	 {
		 for(i = 0; i < 4; i++)
			chassis_move.motor_speed_pid[i].out=0;
	 }
		else if(chassis_behaviour_update->chassis_mode==CHASSIS_INIT)
		{
			chassis_behaviour_update->vx=0;
			chassis_behaviour_update->vy=0;
			chassis_behaviour_update->vz=0;
			mecanum_calc(chassis_behaviour_update->vx,chassis_behaviour_update->vy,\
					chassis_behaviour_update->vz,chassis_behaviour_update->wheel_spd_ref);
			for (i = 0; i < 4; i++)
			{
				PID_ca(&chassis_behaviour_update->motor_speed_pid[i], chassis_behaviour_update->wheel_spd_fdb[i], chassis_behaviour_update->wheel_spd_ref[i]);
			}
		}
		else if(chassis_behaviour_update->chassis_mode==CHASSIS_FOLLOW_GIMBAL)
	{	
		relative=chassis_behaviour_update->chassis_yaw_motor->relative_angle;
//		chassis_deadline_limit_angle(chassis_behaviour_update->chassis_yaw_motor->relative_angle,relative,2,chassis_behaviour_update->chassis_relative_angle_set);
		
		sin_yaw = arm_sin_f32((-chassis_behaviour_update->chassis_yaw_motor->relative_angle)*ANGLE_TO_RAD);
		cos_yaw = arm_cos_f32((-chassis_behaviour_update->chassis_yaw_motor->relative_angle)*ANGLE_TO_RAD);
//		relative=0;
//		cos_yaw=1;
//		sin_yaw=0;
		chassis_behaviour_update->vx=(cos_yaw*chassis_behaviour_update->RC_X_ChassisSpeedRef+sin_yaw*chassis_behaviour_update->RC_Y_ChassisSpeedRef);
		chassis_behaviour_update->vy=(-sin_yaw*chassis_behaviour_update->RC_X_ChassisSpeedRef+cos_yaw*chassis_behaviour_update->RC_Y_ChassisSpeedRef);
		if(flag_g==0)
		{
			  chassis_behaviour_update->chassis_relative_angle_set=0.0f;	
				if(flag_q)
						chassis_behaviour_update->chassis_relative_angle_set=-90.0f;	

				chassis_behaviour_update->Chassis_Gyro_Error=PID_ca(&chassis_behaviour_update->chassis_angle_pid,relative,
																														 chassis_behaviour_update->chassis_relative_angle_set);

				
				if(fabs(chassis_behaviour_update->Chassis_Gyro_Error) > 250.0f)
				{
		#if (Hero_Robot==0)
					Rotation_rate = (MAX_CHASSIS_VR_SPEED - 6200.0f-fabs(chassis_behaviour_update->Chassis_Gyro_Error))*(MAX_CHASSIS_VR_SPEED - 6200.0f-fabs(chassis_behaviour_update->Chassis_Gyro_Error))
					/ (MAX_CHASSIS_VR_SPEED * MAX_CHASSIS_VR_SPEED);	
		#else
					Rotation_rate = (MAX_CHASSIS_VR_SPEED - 6400.0f-fabs(chassis_behaviour_update->Chassis_Gyro_Error))*(MAX_CHASSIS_VR_SPEED - 6400.0f-fabs(chassis_behaviour_update->Chassis_Gyro_Error))
					/ (MAX_CHASSIS_VR_SPEED * MAX_CHASSIS_VR_SPEED);
		#endif
				}
				else 
					Rotation_rate=1;
				
				chassis_behaviour_update->vz = chassis_behaviour_update->Chassis_Gyro_Error;
		
			
	//		chassis_behaviour_update->vz=chassis_behaviour_update->RC_Z_ChassisSpeedRef;
			
			chassis_behaviour_update->vx=Rotation_rate*fp32_constrain(chassis_behaviour_update->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED) ;  //mm/s
			chassis_behaviour_update->vy=Rotation_rate*fp32_constrain(chassis_behaviour_update->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
			chassis_behaviour_update->vz=fp32_constrain(chassis_behaviour_update->vz, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
			
		}
		else	if(flag_g==1)
		{
			chassis_behaviour_update->vx=fp32_constrain(chassis_behaviour_update->RC_X_ChassisSpeedRef, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED) ;  //mm/s
			chassis_behaviour_update->vy=fp32_constrain(chassis_behaviour_update->RC_Y_ChassisSpeedRef, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
			chassis_behaviour_update->vz=0;  //deg/s
		}
		mecanum_calc(chassis_behaviour_update->vx,chassis_behaviour_update->vy,chassis_behaviour_update->vz,chassis_behaviour_update->wheel_spd_ref);
		for (i = 0; i < 4; i++)
		{
			PID_ca(&chassis_behaviour_update->motor_speed_pid[i], chassis_behaviour_update->wheel_spd_fdb[i], chassis_behaviour_update->wheel_spd_ref[i]);
		}
	}
	else if(chassis_behaviour_update->chassis_mode==CHASSIS_DODGE_MODE)
	{
		flag_g=0;
		flag_q=0;
		sin_yaw = arm_sin_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle*ANGLE_TO_RAD);
		cos_yaw = arm_cos_f32(-chassis_behaviour_update->chassis_yaw_motor->relative_angle*ANGLE_TO_RAD);
 
		chassis_behaviour_update->vx=cos_yaw*chassis_behaviour_update->RC_X_ChassisSpeedRef+sin_yaw*chassis_behaviour_update->RC_Y_ChassisSpeedRef;
		chassis_behaviour_update->vy=-sin_yaw*chassis_behaviour_update->RC_X_ChassisSpeedRef+cos_yaw*chassis_behaviour_update->RC_Y_ChassisSpeedRef;
		
		if(GameRobotStat.chassis_power_limit==50) 
			chassis_behaviour_update->vz=vz_50_speed;
		else if(GameRobotStat.chassis_power_limit==55)
			chassis_behaviour_update->vz=vz_55_speed;
		else if(GameRobotStat.chassis_power_limit==60)	
			chassis_behaviour_update->vz=vz_60_speed;
		else if(GameRobotStat.chassis_power_limit==65)	
			chassis_behaviour_update->vz=vz_65_speed;
		else
			chassis_behaviour_update->vz=vz_65_speed;

		if(Shift_Flag==1)
			chassis_behaviour_update->vz=400;

	/**********************************************/
		mecanum_calc(chassis_behaviour_update->vx,chassis_behaviour_update->vy,chassis_behaviour_update->vz,chassis_behaviour_update->wheel_spd_ref);
			for (i = 0; i < 4; i++)
		{
			PID_ca(&chassis_behaviour_update->motor_speed_pid[i], chassis_behaviour_update->wheel_spd_fdb[i], chassis_behaviour_update->wheel_spd_ref[i]);
		}
	}  	

	Chassis_Power_Limit(chassis_behaviour_update);
}

uint16_t max_wheel_speed;		float error;

static void mecanum_calc(float vx, float vy, float vz, float wheel_rpm[])
{

	static float rotate_ratio_fr;
	static float rotate_ratio_fl;
	static float rotate_ratio_bl;
	static float rotate_ratio_br;
	static float wheel_rpm_ratio;
  chassis_move.rotate_x_offset=0;
	chassis_move.rotate_y_offset=0;
	
  rotate_ratio_fr = ((WHEELBASE+WHEELTRACK)/2.0f \
                      - chassis_move.rotate_x_offset + chassis_move.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_fl = ((WHEELBASE+WHEELTRACK)/2.0f \
                      - chassis_move.rotate_x_offset - chassis_move.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_bl = ((WHEELBASE+WHEELTRACK)/2.0f \
                      + chassis_move.rotate_x_offset - chassis_move.rotate_y_offset)/RADIAN_COEF;
  rotate_ratio_br = ((WHEELBASE+WHEELTRACK)/2.0f \
                      + chassis_move.rotate_x_offset + chassis_move.rotate_y_offset)/RADIAN_COEF;
	
  wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);

  float   max = 0;

	wheel_rpm[0] = (+vx - vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;
	wheel_rpm[1] = (+vx + vy + vz * rotate_ratio_fl) * wheel_rpm_ratio;
	wheel_rpm[2] = (-vx + vy + vz * rotate_ratio_bl) * wheel_rpm_ratio;
	wheel_rpm[3] = (-vx - vy + vz * rotate_ratio_br) * wheel_rpm_ratio;

	if(Ctrl_Flag == 1)
	{
		max_wheel_speed=1500;
	}	
	else if((Super_power.input_volt < 17000||Shift_Flag == 2) && Ctrl_Flag==0)
	{
		if(GameRobotStat.chassis_power_limit==50) 
			max_wheel_speed=power_50_speed;
		if(GameRobotStat.chassis_power_limit==55)
			max_wheel_speed=power_55_speed;
		if(GameRobotStat.chassis_power_limit==60)	
			max_wheel_speed=power_60_speed;
		if(GameRobotStat.chassis_power_limit==65)	
			max_wheel_speed=power_65_speed;
		if(GameRobotStat.chassis_power_limit==70)
			max_wheel_speed=power_70_speed;
		if(GameRobotStat.chassis_power_limit==90)	
			max_wheel_speed=power_90_speed;
		if(GameRobotStat.chassis_power_limit==120)
			max_wheel_speed=power_120_speed;
 
		if(GameRobotStat.chassis_power_limit!=50
			&&GameRobotStat.chassis_power_limit!=55
			&&GameRobotStat.chassis_power_limit!=60
			&&GameRobotStat.chassis_power_limit!=65
			&&GameRobotStat.chassis_power_limit!=70
			&&GameRobotStat.chassis_power_limit!=90
			&&GameRobotStat.chassis_power_limit!=120) 
				max_wheel_speed=power_50_speed;//55w
			//50w 4500
			//55w 5000
			//60w 6150
			//65W 6700
	}
	else if(Shift_Flag==1)
	{
			max_wheel_speed = 10000;
	}	
		
	 for (uint8_t i = 0; i < 4; i++)
  {
		if (fabs(wheel_rpm[i]) > max)
		max = fabs(wheel_rpm[i]);
  }
  //equal proportion
	if (max > max_wheel_speed)
  {
    float rate = max_wheel_speed / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
	error=gimbal_control.gimbal_pitch_motor.absolute_angle-gimbal_control.gimbal_pitch_motor.relative_angle;
	if(fabs(gimbal_control.gimbal_pitch_motor.absolute_angle-gimbal_control.gimbal_pitch_motor.relative_angle)>12.0f&&Shift_Flag!=1)
	{
//		wheel_rpm[0]=fp32_constrain(wheel_rpm[0],-max_wheel_speed/3,max_wheel_speed/3);
//		wheel_rpm[1]=fp32_constrain(wheel_rpm[1],-max_wheel_speed/3,max_wheel_speed/3);
		wheel_rpm[0]=wheel_rpm[0]/2.5f;
		wheel_rpm[1]=wheel_rpm[1]/2.5f;
		wheel_rpm[2]=wheel_rpm[2]*1.5f;
		wheel_rpm[3]=wheel_rpm[3]*1.5f;
	}
}
/*********************************************************************************************************************************************************************/
static void chassis_give_current(chassis_move_t *chassis_current)
{
	static int i;
	for(i=0;i<4;i++)
	{ //赋值电流值
		chassis_current->give_current[i] = (int16_t)(chassis_current->motor_speed_pid[i].out);
	}
}
	float    Joule_Residue = 0.0f;//剩余焦耳缓冲能量

void Chassis_Power_Limit(chassis_move_t *chassis_limit)
{	
	/*********************祖传算法*************************/
	float    kLimit = 0.0f;//功率限制系数
	float 	 fTotalCurrentLimit;
	float    chassis_totaloutput = 0.0f;//统计总输出电流
	static 	 int16_t judgDataError_Time = 0;

	Joule_Residue =PowerHeatData.chassis_power_buffer;//剩余焦耳能量	

	//统计底盘总输出
	chassis_totaloutput = abs((int16_t)chassis_limit->motor_speed_pid[0].out) + abs((int16_t)chassis_limit->motor_speed_pid[1].out)+
						  abs((int16_t)chassis_limit->motor_speed_pid[2].out) + abs((int16_t)chassis_limit->motor_speed_pid[3].out);
	
	if(chassis_limit->chassis_monitor_point[JudgementWDG].errorExist == 1)//裁判系统无效时强制限速
	{
		judgDataError_Time++;
		if(judgDataError_Time > 100)
		{
			for (u8 i = 0; i < 4; i++)
			{
				VAL_LIMIT(chassis_limit->motor_speed_pid[i].out, -1000, 1000);//
			}
		}
	}
	if(chassis_limit->chassis_monitor_point[JudgementWDG].errorExist == 0)
	{
		judgDataError_Time = 0;
		//剩余焦耳量过小,开始限制输出,限制系数为平方关系
		if(Joule_Residue < WARNING_REMAIN_POWER)
		{
			kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
						* (float)(Joule_Residue / WARNING_REMAIN_POWER);
			fTotalCurrentLimit =(kLimit * MAX_CHASSIS_OUT * 4);
		}
		else //焦耳能量恢复到一定数值
		{
			fTotalCurrentLimit = (MAX_CHASSIS_OUT * 4);
		}
		
		//底盘各电机电流重新分配
		if (chassis_totaloutput > fTotalCurrentLimit)
		{
		//赋值电流值
			chassis_limit->motor_speed_pid[0].out = ((chassis_limit->motor_speed_pid[0].out) / chassis_totaloutput * fTotalCurrentLimit);
			chassis_limit->motor_speed_pid[1].out = ((chassis_limit->motor_speed_pid[1].out) / chassis_totaloutput * fTotalCurrentLimit);
			chassis_limit->motor_speed_pid[2].out = ((chassis_limit->motor_speed_pid[2].out) / chassis_totaloutput * fTotalCurrentLimit);
			chassis_limit->motor_speed_pid[3].out = ((chassis_limit->motor_speed_pid[3].out) / chassis_totaloutput * fTotalCurrentLimit);
		}
	}
		chassis_give_current(chassis_limit);
}

void Super_power_ctrl(chassis_move_t *power_ctrl)
{
	uint16_t fTotalpowerLimit;
	
	if(power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 0)
	{
		fTotalpowerLimit = GameRobotStat.chassis_power_limit;
//		if(Joule_Residue<22)
//			CAN_CMD_SUPERPOWER(fTotalpowerLimit ,Shift_Flag);
//		else 
//			CAN_CMD_SUPERPOWER(fTotalpowerLimit+5,Shift_Flag);
				
		CAN_CMD_SUPERPOWER(fTotalpowerLimit ,Shift_Flag);
	}
	else if(power_ctrl->chassis_monitor_point[JudgementWDG].errorExist == 1)
	{
		CAN_CMD_SUPERPOWER(20 ,Shift_Flag);//20W充电使用
	}
}


