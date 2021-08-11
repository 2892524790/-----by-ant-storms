#include "GIMBAL_TASK.h"
#include "SHOOT_TASK.h"
#include "Tx2_task.h"
#include "chassis_task.h"
#include "RM_Client_UI.h"
#include "can_receive.h"
extern Shoot_t shoot;
extern first_order_filter_type_t rc_filter;
//first_order_filter_type_t Glimbal_Yaw_filter;
extern ext_robot_hurt_t	RobotHurt;					//0x0206
extern PC_send_measure_t SEND_PC;
//云台控制所有相关数据
Gimbal_Control_t gimbal_control;
char press_r,mouse_r; //自瞄标志位
static char last_flag=0;
char key_z,key_x,key_c;
 
extern int FFlag_state;
float flash_dat;
extern char flag_b;

////云台初始化
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//云台电机返回的数据
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//云台模式设定
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);

static void gimbal_relax(Gimbal_Control_t *gimbal_motor);
static void gimbal_init(Gimbal_Control_t *gimbal_motor);
static void gimbal_absolute_control(Gimbal_Control_t *gimbal_motor);
static void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_track_control(Gimbal_Control_t *gimbal_motor);

void gimbal_task(void *pvParameters)
{ 
  vTaskDelay(GIMBAL_TASK_INIT_TIME);
	GIMBAL_Init(&gimbal_control);
	#if detect

		 //判断电机是否都上线
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE))
    {
        vTaskDelay(1);
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
    }
	#endif

	for(;;)
	{
  
		GIMBAL_Set_Mode(&gimbal_control);
		GIMBAL_Feedback_Update(&gimbal_control);
		GIMBAL_Set_Contorl(&gimbal_control);  
 
	if(fabs(gimbal_control.gimbal_yaw_motor.gimbal_motor_angle_pid.error[0])<1.0f&&(key_z==0||key_x==0||key_c==0))
	{
		if(!IF_KEY_PRESSED_Z)
			key_z=1;	
		if(!IF_KEY_PRESSED_X)
			key_x=1;
		if(!IF_KEY_PRESSED_C)
			key_c=1;
		gimbal_control.yaw_angle_dynamic_ref=gimbal_control.gimbal_yaw_motor.absolute_angle;
		
	}
		
		gimbal_control.yaw_current = gimbal_control.gimbal_yaw_motor.given_current;
		gimbal_control.pit_current = gimbal_control.gimbal_pitch_motor.given_current;
#if detect

		//电机未掉线 检测遥控是否掉线 如果掉线云台停止
        if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE)))
				{			
	        if (toe_is_error(DBUSTOE))
            {
                CAN_CMD_GIMBAL(0, 0, shoot.dirver_give_current, 0);
            }
            else
            {
							CAN_CMD_GIMBAL(gimbal_control.pit_current,gimbal_control.yaw_current , shoot.dirver_give_current, 0);
            }
					}		
#else
	CAN_CMD_GIMBAL(gimbal_control.pit_current,gimbal_control.yaw_current ,shoot.dirver_give_current, 0);
	
	
#endif
		vTaskDelay(1);
	}
}

//云台初始化
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
		 if (gimbal_init == NULL)
    {
        return;
    }
		gimbal_init->gimbal_mode = GIMBAL_RELAX;
	//遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
		//PC数据指针获取
    gimbal_init->PC_Ctrl_Measure_Point = get_PC_Ctrl_Measure_Point();
		//陀螺仪数据指针获取
		gimbal_init->gimbal_angle_gyro_point = get_Gyro_Angle_Point();
		gimbal_init->gimbal_monitor_point = getErrorListPoint();
		KalmanCreate(&gimbal_init->gimbal_pitch_motor.Error_Kalman, 50, 20);
		KalmanCreate(&gimbal_init->gimbal_yaw_motor.Error_Kalman, 50, 20);
		//电机数据指针获取
		gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
		gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
			//归中初始化
		gimbal_init->gimbal_pitch_motor.offset_ecd = Glimbal_Pitch_Offset;
		gimbal_init->gimbal_yaw_motor.offset_ecd = Glimbal_Yaw_Offset;
		//yaw角度环 
		PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_angle_pid, PID_POSITION,30000, 2000, 90, 0, 0);
		//yaw速度
		PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid,PID_POSITION,30000,18000,20, 0.0f, 0);
		//pit角度环 
		PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_angle_pid,PID_POSITION, 30000, 2000, 120, 0, 0);
		//pit速度环 
		PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid,PID_POSITION,30000,18000,30, 0, 0);
		
//		first_order_filter_init(&Glimbal_Yaw_filter,0.0002f,0.008f);

}
//云台模式设置
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
	 if(gimbal_set_mode == NULL)
    {
        return;
    }
		gimbal_set_mode->last_gimbal_mode=gimbal_set_mode->gimbal_mode;
		
		if(switch_is_down (gimbal_set_mode->gimbal_rc_ctrl->rc.s[1])&&switch_is_up(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0]))
		{
			gimbal_set_mode->gimbal_mode =GIMBAL_FOLLOW_MODE;	
			last_flag=mouse_r;
			
      if(gimbal_set_mode->gimbal_rc_ctrl->mouse.press_r&&press_r==0)
			{
				press_r=1;
				mouse_r=!mouse_r;
				flag_b=0;

			}
			else if(!gimbal_set_mode->gimbal_rc_ctrl->mouse.press_r)
					press_r=0;
			/*
			自瞄与转头方案测试:
			1.转头时再次进入pid切换，且并不改期望  
			2.分离自瞄切换与转头
			*/
			if(last_flag!=mouse_r/*||last_work_turn!=work_turn*/)
			{	//自瞄切换pid
				if(mouse_r==0)
				{
					pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_angle_pid, yaw_follow_angle_kp,0,0);
					pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_gyro_pid,yaw_follow_speed_kp,0,0);
					pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_angle_pid,pitch_follow_angle_kp,pitch_follow_angle_ki,0);
					pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_gyro_pid,pitch_follow_speed_kp,0,0);
					
			
				}
				 if(mouse_r==1)
				{
					pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_angle_pid, yaw_track_angle_kp,yaw_track_angle_ki,0);
					pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_gyro_pid,yaw_track_speed_kp,0,0);
					pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_angle_pid,pitch_track_angle_kp,pitch_track_angle_ki,pitch_track_angle_kd);
					pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_gyro_pid,pitch_track_speed_kp,0,0);
					
				}
					gimbal_set_mode->yaw_angle_dynamic_ref=gimbal_set_mode->gimbal_yaw_motor.absolute_angle;
					gimbal_set_mode->pitch_angle_dynamic_ref=gimbal_set_mode->gimbal_pitch_motor.absolute_angle;
					key_z=key_x=key_c=1;
			}
		
		}
		else 
		{
					key_z=key_x=key_c=1;
					press_r=0;
					mouse_r=0;
					flag_b=0;

				if (switch_is_down(gimbal_set_mode->gimbal_rc_ctrl->rc.s[1])&&switch_is_down(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0]))
				{//左下右下 无力
						gimbal_set_mode->gimbal_mode = GIMBAL_RELAX;
				}
				else if (switch_is_down(gimbal_set_mode->gimbal_rc_ctrl->rc.s[1])&&switch_is_mid(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0]))
				{//左下右中 初始化
						gimbal_set_mode->gimbal_mode =  GIMBAL_INIT;
				}
				else if (	(switch_is_mid(gimbal_set_mode->gimbal_rc_ctrl->rc.s[1])||switch_is_up(gimbal_set_mode->gimbal_rc_ctrl->rc.s[1]))	&&  
										(switch_is_mid(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0])||switch_is_up(gimbal_set_mode->gimbal_rc_ctrl->rc.s[0]) ) )
				{//当左在中,跟随模式
						gimbal_set_mode->gimbal_mode =GIMBAL_FOLLOW_MODE; 
				} 
				else 
					gimbal_set_mode->gimbal_mode = GIMBAL_RELAX;
		}
		

    if(gimbal_set_mode->gimbal_mode==GIMBAL_INIT&&gimbal_set_mode->last_gimbal_mode!=GIMBAL_INIT)
		{
			pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_angle_pid,yaw_init_angle_kp,0,0);
			pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_gyro_pid,yaw_init_speed_kp,0 ,0);
			pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_angle_pid, pitch_init_angle_kp,0,0);
			pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_gyro_pid,pitch_init_speed_kp,0,0);
		}
		if(gimbal_set_mode->gimbal_mode==GIMBAL_FOLLOW_MODE&&gimbal_set_mode->last_gimbal_mode!=GIMBAL_FOLLOW_MODE)
		{
			//***************************************陀螺仪PID控制**************************************
			pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_angle_pid, yaw_follow_angle_kp,0,0);
			pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_gyro_pid,yaw_follow_speed_kp,0,0);
			pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_angle_pid,pitch_follow_angle_kp,pitch_follow_angle_ki,0);
			pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_gyro_pid,pitch_follow_speed_kp,0,0);
			//***************************************编码器PID控制*******************************************
//					pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_angle_pid, 38,0,0);
	//					pid_reset(&gimbal_set_mode->gimbal_yaw_motor.gimbal_motor_gyro_pid,225,0,0);
	//					pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_angle_pid,21,0,0);
	//					pid_reset(&gimbal_set_mode->gimbal_pitch_motor.gimbal_motor_gyro_pid,250,0,0);
		}
	
}
//云台数据更新
fp32 yaw_channel = 0,pitch_channel = 0;
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
			
 
		gimbal_feedback_update->gimbal_yaw_motor.offset_ecd = Glimbal_Yaw_Offset;
 		
    //云台数据更新
#if (Hero_Robot==0)
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = -(gimbal_feedback_update->gimbal_angle_gyro_point->Pitch);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = get_relative_pos(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd)/ENCODER_ANGLE;
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro =(gimbal_feedback_update->gimbal_angle_gyro_point->V_Y);
#else 
		 gimbal_feedback_update->gimbal_pitch_motor.absolute_angle =(gimbal_feedback_update->gimbal_angle_gyro_point->Pitch);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = get_relative_pos(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd)/ENCODER_ANGLE;
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro =-(gimbal_feedback_update->gimbal_angle_gyro_point->V_Y);

		
#endif
  
		gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = -(gimbal_feedback_update->gimbal_angle_gyro_point->YAW);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle =(get_relative_pos(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd)/ENCODER_ANGLE);
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = (gimbal_feedback_update->gimbal_angle_gyro_point->V_Z);
		if(fabs(gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_pitch)>8.0f)
		gimbal_feedback_update->pitch_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_pitch*0.5f;
		else
		gimbal_feedback_update->pitch_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_pitch;
		if(fabs(gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_yaw)>8.0f)
		gimbal_feedback_update->yaw_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_yaw*0.5f;
		else
		gimbal_feedback_update->yaw_angle_PcCtrl_ref = gimbal_feedback_update->PC_Ctrl_Measure_Point->PcDate.angle_yaw;
				
		//动态输入角度更新
		yaw_channel=gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[2];
		pitch_channel=gimbal_feedback_update->gimbal_rc_ctrl->rc.ch[3];
		
	
#if (Hero_Robot==0)
	if(switch_is_down(gimbal_feedback_update->gimbal_rc_ctrl->rc.s[1])&&switch_is_up(gimbal_feedback_update->gimbal_rc_ctrl->rc.s[0]))
		{
			gimbal_feedback_update->pitch_angle_dynamic_ref+=fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.y,-120,120)*kKey_Gyro_Pitch;//0.01
			gimbal_feedback_update->yaw_angle_dynamic_ref-=fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.x,-120,120)*kKey_Gyro_Yaw;//0.005
		}
		else 
		{	
				gimbal_feedback_update->pitch_angle_dynamic_ref -= pitch_channel*STICK_TO_PITCH_ANGLE_INC_FACT;
				gimbal_feedback_update->yaw_angle_dynamic_ref -= yaw_channel*STICK_TO_YAW_ANGLE_INC_FACT;
		}
#else
			if(switch_is_down(gimbal_feedback_update->gimbal_rc_ctrl->rc.s[1])&&switch_is_up(gimbal_feedback_update->gimbal_rc_ctrl->rc.s[0]))
		{
			gimbal_feedback_update->pitch_angle_dynamic_ref-=fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.y,-120,120)*kKey_Gyro_Pitch;//0.01
			gimbal_feedback_update->yaw_angle_dynamic_ref-=fp32_constrain(gimbal_feedback_update->gimbal_rc_ctrl->mouse.x,-120,120)*kKey_Gyro_Yaw;//0.005
		}
		else 
		{	
			gimbal_feedback_update->pitch_angle_dynamic_ref += pitch_channel*STICK_TO_PITCH_ANGLE_INC_FACT;
			gimbal_feedback_update->yaw_angle_dynamic_ref -= yaw_channel*STICK_TO_YAW_ANGLE_INC_FACT;
		}
#endif			
}
//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
	if(gimbal_set_control->gimbal_mode==GIMBAL_INIT)
		gimbal_init(gimbal_set_control);
	else if(gimbal_set_control->gimbal_mode==GIMBAL_FOLLOW_MODE)
	{
		if(mouse_r==0)
			gimbal_absolute_control(gimbal_set_control);
		else if(mouse_r==1)
			gimbal_track_control(gimbal_set_control);
	}
	else
		gimbal_relax(gimbal_set_control);
}
static void gimbal_relax(Gimbal_Control_t *gimbal_motor)
{
	gimbal_motor->gimbal_yaw_motor.given_current=0;
	gimbal_motor->gimbal_pitch_motor.given_current=0;
	gimbal_motor->yaw_angle_dynamic_ref=0;
	gimbal_motor->pitch_angle_dynamic_ref=0;
}
static void gimbal_init(Gimbal_Control_t *gimbal_motor)
{
	gimbal_motor->gimbal_pitch_motor.gimbal_angle_set=0;
	gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);

	gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=0;
  gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor);
	gimbal_motor->yaw_angle_dynamic_ref=0;
	gimbal_motor->pitch_angle_dynamic_ref=0;
	
}
static void gimbal_absolute_control(Gimbal_Control_t *gimbal_motor)
{

	if(key_z&&key_x&&key_c)
	{
		if(fabs(gimbal_motor->yaw_angle_dynamic_ref)==0)
			gimbal_motor->yaw_angle_dynamic_ref=gimbal_motor->gimbal_yaw_motor.absolute_angle;
		else
			VAL_LIMIT(gimbal_motor->yaw_angle_dynamic_ref,  gimbal_motor->gimbal_yaw_motor.absolute_angle -10.0f, 
																											gimbal_motor->gimbal_yaw_motor.absolute_angle +10.0f);
		
			gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->yaw_angle_dynamic_ref;
	}
	
	VAL_LIMIT(gimbal_motor->pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX);

	gimbal_motor->gimbal_pitch_motor.gimbal_angle_set=gimbal_motor->pitch_angle_dynamic_ref;
	
	if(IF_KEY_PRESSED_Z&&key_z)
	{
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.absolute_angle+150;
		key_z=0;
	}
	else if(IF_KEY_PRESSED_X&&key_x)
	{
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.absolute_angle+70;
		key_x=0;
	}
	else if(IF_KEY_PRESSED_C&&key_c)
	{
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.absolute_angle-70;
		key_c=0;
	}
	
	gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
	gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
	
//	gimbal_relative_angle_control(&gimbal_motor->gimbal_pitch_motor);
//  gimbal_relative_angle_control(&gimbal_motor->gimbal_yaw_motor); 
}

fp32 yawrate=0.015f;
fp32 pitrate=0.0045f;
static void gimbal_track_control(Gimbal_Control_t *gimbal_motor)
{
	if(toe_is_error(TX2DataTOE)==1)
	{
#if (Hero_Robot==0)

		gimbal_motor->gimbal_pitch_motor.gimbal_angle_set+=fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y,-90,90)*kKey_Gyro_Pitch;
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set -=fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x,-120,120)*kKey_Gyro_Yaw;
#else

		gimbal_motor->gimbal_pitch_motor.gimbal_angle_set-=fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.y,-90,90)*kKey_Gyro_Pitch;
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set -=fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x,-120,120)*kKey_Gyro_Yaw;
#endif
		//*****************************************视觉数据切换清i**************************************
		pid_i_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_angle_pid);
		pid_i_reset(&gimbal_motor->gimbal_yaw_motor.gimbal_motor_angle_pid);
		pid_i_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid);
		pid_i_reset(&gimbal_motor->gimbal_pitch_motor.gimbal_motor_gyro_pid);
		
		
	}
	else 
	{
		gimbal_motor->gimbal_pitch_motor.gimbal_angle_set =  
												+gimbal_motor->pitch_angle_PcCtrl_ref+gimbal_motor->gimbal_pitch_motor.absolute_angle;
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set =  
											  +gimbal_motor->yaw_angle_PcCtrl_ref*1.1f+gimbal_motor->gimbal_yaw_motor.absolute_angle-fp32_constrain(gimbal_motor->gimbal_rc_ctrl->mouse.x,-90,90)*0.03;
	}
								

	if(IF_KEY_PRESSED_Z&&key_z)
	{
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.absolute_angle+150;
		key_z=0;
	}
	else if(IF_KEY_PRESSED_X&&key_x)
	{
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.absolute_angle+70;
		key_x=0;
	}
	else if(IF_KEY_PRESSED_C&&key_c)
	{
		gimbal_motor->gimbal_yaw_motor.gimbal_angle_set=gimbal_motor->gimbal_yaw_motor.absolute_angle-70;
		key_c=0;
	}
 
	gimbal_absolute_angle_control(&gimbal_motor->gimbal_yaw_motor);
	gimbal_absolute_angle_control(&gimbal_motor->gimbal_pitch_motor);
}
//*******************************************************************************************************************************************************************/
static void gimbal_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
	{
		if (gimbal_motor == NULL)
		{
				return;
		}
	//角度环，速度环串级pid调试
		gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_angle_pid, gimbal_motor->relative_angle, gimbal_motor->gimbal_angle_set,gimbal_motor->Error_Kalman,0);
		gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->gimbal_motor_measure->speed_rpm ,gimbal_motor->motor_gyro_set,gimbal_motor->Error_Kalman,0);
		//控制值赋值
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
static void gimbal_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
	  //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_angle_pid,gimbal_motor->absolute_angle, gimbal_motor->gimbal_angle_set,gimbal_motor->Error_Kalman,0);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set,gimbal_motor->Error_Kalman,0);

		//控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		
	 
}
/*********************************************************************************************************************************************************************/
const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}
//char key_b=0;
//	u16 write_count=0;
//	FLASH_Status write,erase;
//		union flash_keep
//		{
//			uint32_t b;
//			float a;
//		}keepdate;
//		char flag_flash=0;
//fp32 keep_flash(fp32 pitch)
//{

//		keepdate.a=pitch;
//			if(IF_KEY_PRESSED_B)
//		{ 
//			
//			FLASH_Unlock();
//			do
//			erase=FLASH_EraseSector(FLASH_Sector_10, VoltageRange_3);
//			while(erase!=FLASH_COMPLETE);
//			do
//			{
//				write=FLASH_ProgramWord(0x80c0000, keepdate.b);
//			  vTaskDelay(1);
//			}
//			while(write!=FLASH_COMPLETE);
//			FLASH_Lock();
//		}
//		return *(fp32*)0x80c0000;
//}

