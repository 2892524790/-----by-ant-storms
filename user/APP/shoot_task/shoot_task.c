#include "SHOOT_TASK.h"
#include "start_task.h"
#include "gimbal_task.h"
#include "judge_task.h"
#include "remote_control.h"
#include "adc.h"
#include "led.h" 
#include "main.h"

Shoot_t shoot;
char key_e = 0, flag_e = 0;
char key_r = 0, flag_r = 0;
//char key_q = 0, flag_q = 0;
char open_flag=0;
fp32 close_angle;

static void shoot_init(Shoot_t* shoot_init);
//射击模式
static void shoot_mode(Shoot_t* shoot_mode);
//射击控制及其赋值
static void shoot_contorl(Shoot_t* shoot_control);
//射击热量限制
static void shoot_limit(Shoot_t* shoot_limit);
//射击电流
static void shoot_give_current(Shoot_t* shoot_give_current);
extern ext_robot_hurt_t					RobotHurt;					//0x0206

extern ext_power_heat_data_t PowerHeatData;
extern ext_game_robot_state_t GameRobotStat;
extern Encoder_process_t Encoder_friction[2];

int32_t fri_spdset;
//u16 speed_ref10 = 4320;
//u16 speed_ref12 = 4900;
//u16 speed_ref14 = 5650;
//u16 speed_ref16 = 6100;



#if (Hero_Robot==0)

	#if (version_ball == 0)

			#define dirver_speed_kp 15
			#define fri_speed_kp 200
			#define fri3_speed_kp 40
			#define fri3_angle_kp 1.1f
			

			int big_angle = -90;
			#define small_angle -2.785f
			u16 speed_ref10 = 510;//580(极限);//590 595;
			//u16 speed_ref12 = 595;
			//u16 speed_ref14 = 595;
			u16 speed_ref16 = 900;
			#define is_ready	!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)

	#else //新

			#define dirver_speed_kp 15
			#define fri_speed_kp 200
			#define fri3_speed_kp 40
			#define fri3_angle_kp 1.1f

			int big_angle = -80;
			#define small_angle -2.785f
			u16 speed_ref10 = 510;//580(极限);//590 595;
			//u16 speed_ref12 = 595;
			//u16 speed_ref14 = 595;
			u16 speed_ref16 = 870;
			float adc=0.4;
			#define is_ready  ((Get_Adc_Average(ADC_Channel_1,5)*3.3f/4096)<adc)//!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
	#endif

#else //新英雄
	#if (version_ball == 0)

		#define dirver_speed_kp 25
		#define fri_speed_kp 195
		#define fri3_speed_kp 45
		#define fri3_angle_kp 1.1f

		int big_angle=75;
		float small_angle=3.785f;
			u16 speed_ref10 = 540;//580(极限);//590 595;
		//u16 speed_ref12 = 595;
		//u16 speed_ref14 = 595;
		u16 speed_ref16 = 775;

		#define is_ready 		!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
		
	#else
		#define dirver_speed_kp 15
		#define fri_speed_kp 195
		#define fri3_speed_kp 35
		#define fri3_angle_kp 1.1f

		int big_angle=45;
		float small_angle=3.785f;
			u16 speed_ref10 = 530;//580(极限);//590 595;
		//u16 speed_ref12 = 595;
		//u16 speed_ref14 = 595;
		u16 speed_ref16 = 800;
		float adc=0.08f;

		#define is_ready  !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)//((Get_Adc(ADC_Channel_1)*3.3f/4096)<adc)//

	#endif
#endif
float abc;
void Shoot_task(void* pvParameters)
{
    vTaskDelay(200);
    shoot_init(&shoot);
#if detect

    while(toe_is_error(frictionmotorRTOE) || toe_is_error(frictionmotorLTOE) || toe_is_error(frictionmotorATOE) || toe_is_error(TriggerMotorTOE) || toe_is_error(TriggerMotorTOE))
    {
        vTaskDelay(3);
    }
#endif
    for(;;)
    {
        shoot_mode(&shoot);
        shoot_contorl(&shoot);
//#if	version_ball==1
////			abc= Get_Adc(ADC_Channel_1)*3.3f/4096;
//#endif
#if detect

        if(!(toe_is_error(frictionmotorRTOE) && toe_is_error(frictionmotorLTOE) && toe_is_error(TriggerMotorTOE)))
        {
            if(toe_is_error(DBUSTOE))
            {
                CAN_CMD_SHOOT(0, 0, 0, 0);
            }
            else
                CAN_CMD_SHOOT(shoot.dirver_give_current, shoot.shoot_give_current[0], shoot.shoot_give_current[1], shoot.shoot_give_current[2]);

        }
#else
        CAN_CMD_SHOOT(shoot.dirver_give_current, shoot.shoot_give_current[0], shoot.shoot_give_current[1], shoot.shoot_give_current[2]);
#endif
        vTaskDelay(1);
    }
}
static void shoot_init(Shoot_t* shoot_init)
{

    shoot_init->shoot_mode = shoot_not_fire;
		shoot_init->shoot_RC=get_remote_control_point();

    shoot_init->motor_friction1.shoot_motor_measure = get_Friction1__Motor_Measure_Point();
    shoot_init->motor_friction2.shoot_motor_measure = get_Friction2__Motor_Measure_Point();
    shoot_init->motor_friction3.shoot_motor_measure = get_Friction3__Motor_Measure_Point();
    shoot_init->motor_dirver.dirver_motor_measure = get_Dirver__Motor_Measure_Point();
		shoot_init->motor_2006.shoot_motor_measure=get_2006__Motor_Measure_Point();
	
    PID_init(&shoot_init->dirver_speed_pid, PID_POSITION, 16000, 2000, dirver_speed_kp, 0, 0);	// 15 0.15

    PID_init(&shoot_init->friction1_speed_pid, PID_POSITION, 9000, 0, fri_speed_kp, 0, 0);
    PID_init(&shoot_init->friction2_speed_pid, PID_POSITION, 9000, 0, fri_speed_kp, 0, 0);
    PID_init(&shoot_init->friction3_speed_pid, PID_POSITION, 16000, 2000,fri3_speed_kp, 0, 0);
		PID_init(&shoot_init->friction3_angle_pid, PID_POSITION, 8000, 2000,fri3_angle_kp, 0, 0);

		shoot_init->friction3_angle_pid.set = shoot_init->motor_friction3.shoot_motor_measure->real_ecd; 

}
static void shoot_mode(Shoot_t* shoot_mode)
{
    if(shoot_mode == NULL)
    {
        return;
    }

    if(switch_is_down(shoot_mode->shoot_RC->rc.s[1]) && switch_is_up(shoot_mode->shoot_RC->rc.s[0]))
    {
        if(IF_KEY_PRESSED_E && key_e == 0)
        {
            key_e = 1;
            flag_e = !flag_e;
        }
        else if(!IF_KEY_PRESSED_E)
            key_e = 0;
        if(flag_e == 0)
				{
					shoot_mode->shoot_mode = shoot_not_fire;
					flag_r=0;
				}
        else if(flag_e == 1)
            shoot_mode->shoot_mode = shoot_fire;
    }
    else 
    {
			flag_e=0;
			flag_r=0;
      if(switch_is_up(shoot_mode->shoot_RC->rc.s[1]) &&
				(switch_is_mid(shoot_mode->shoot_RC->rc.s[0]) || switch_is_up(shoot_mode->shoot_RC->rc.s[0])))
			{
				shoot_mode->shoot_mode = shoot_fire;
			}
			else if((shoot_mode->friction3_speed_pid.fdb==0)&&(shoot_mode->dirver_speed_pid.fdb==0))
			{
				shoot_mode->shoot_mode = shoot_not_fire;
				open_flag=0;
			}
      else if (switch_is_down(shoot_mode->shoot_RC->rc.s[0]) && switch_is_down(shoot_mode->shoot_RC->rc.s[1]))
			{
				shoot_mode->shoot_mode = shoot_not_fire;
			}
    }
}

static void shoot_limit(Shoot_t* shoot_limit)
{
    static fp32 HP_loss = 0;
    static uint16_t next_heat = 0;
		static int i=0;
	
		if(IF_KEY_PRESSED_R && key_r == 0)
			{
					key_r = 1;
					flag_r = !flag_r;
			}
			else if(!IF_KEY_PRESSED_R)
					key_r = 0;

    shoot_limit->remaining_shoot_heat = (GameRobotStat.shooter_id1_42mm_cooling_limit - PowerHeatData.shooter_id1_42mm_cooling_heat);
		
		if(i&&RobotHurt.hurt_type ==3)				//0x0206
		{
//			i++;
//			shoot_limit->remaining_shoot_heat = 0;
//			if(i>=200)
//				i=1;
		}
		
    if(flag_r || shoot_limit->remaining_shoot_heat >= 100 || shoot_limit->shoot_RC->rc.ch[4] == 660)
        shoot_limit->shoot_flag = 1;
    else if(shoot_limit->remaining_shoot_heat < 100)
        shoot_limit->shoot_flag = 0;
		
	
    if(shoot_limit->remaining_shoot_heat < 100 && GameRobotStat.remain_HP < 30)//血量过低，强制停止发弹
        shoot_limit->shoot_flag = 0;
		
		
//*******************************************************************************************************************************************
    next_heat = PowerHeatData.shooter_id1_42mm_cooling_heat + 100; //下一发子弹热量
    if(next_heat > GameRobotStat.shooter_id1_42mm_cooling_limit) //下一发子弹是否超热，计算下一发子弹自发射到热量冷却完成的扣除血量
    {
        HP_loss = 0;
        if(next_heat > GameRobotStat.shooter_id1_42mm_cooling_limit * 2)
        {
            HP_loss += (next_heat - 2 * GameRobotStat.shooter_id1_42mm_cooling_limit) * GameRobotStat.max_HP / 250;
            next_heat = GameRobotStat.shooter_id1_42mm_cooling_limit * 2;
        }
        while(next_heat > GameRobotStat.shooter_id1_42mm_cooling_limit)
        {
            HP_loss += (next_heat - GameRobotStat.shooter_id1_42mm_cooling_limit) * GameRobotStat.max_HP / 2500;
            next_heat -= GameRobotStat.shooter_id1_42mm_cooling_rate / 10;
        }

        if(HP_loss > GameRobotStat.remain_HP-50)
            shoot_limit->heat_flag = 1;	//下一发子弹将导致机器人扣血至50以下
        else shoot_limit->heat_flag = 0;
    }
    else
        shoot_limit->heat_flag = 0;

    if(shoot_limit->heat_flag == 1)//超超热量立即停止发射
        shoot_limit->shoot_flag = 0;


    if(GameRobotStat.shooter_id1_42mm_speed_limit == 10)
    {
        fri_spdset = speed_ref10;//RAMP_float(speed_ref10, fri_spdset, FRI_RAMP_RATE);
    }
//    else if(GameRobotStat.shooter_id1_42mm_speed_limit == 12)
//    {
//        fri_spdset = speed_ref12;//RAMP_float(speed_ref12, fri_spdset, FRI_RAMP_RATE);
//    }
//    else if(GameRobotStat.shooter_id1_42mm_speed_limit == 14)
//    {
//        fri_spdset = speed_ref14;//RAMP_float(speed_ref14, fri_spdset, FRI_RAMP_RATE);
//    }
    else if(GameRobotStat.shooter_id1_42mm_speed_limit == 16)
    {
        fri_spdset =speed_ref16;// RAMP_float(speed_ref16, fri_spdset, FRI_RAMP_RATE);
    }
    else
        fri_spdset = speed_ref16;

}

uint32_t ii;
static void shoot_contorl(Shoot_t* shoot_control)
{
  	static char flag_fire=0,key_fire=0,flag_done=1;
	
    if(shoot_control == NULL)
    {
        return;
    }
    if(shoot_control->shoot_mode == shoot_not_fire)
    {
			shoot_control->dirver_speed_pid.set= 0;
			shoot_control->friction1_speed_pid.set=0;
			shoot_control->friction2_speed_pid.set=0;
			shoot_control->friction3_speed_pid.set=0;
			shoot_give_current(shoot_control);
	
			if(switch_is_down(shoot_control->shoot_RC->rc.s[1]) && switch_is_down(shoot_control->shoot_RC->rc.s[0])
				&&shoot_control->friction3_speed_pid.fdb==0&&(shoot_control->dirver_speed_pid.fdb==0))
			{
				shoot.shoot_give_current[0]=0;
				shoot.shoot_give_current[1]=0;
				shoot.shoot_give_current[2]=0;
				shoot_control->friction3_angle_pid.set = shoot_control->motor_friction3.shoot_motor_measure->real_ecd;
			}
				
			if(switch_is_down(shoot_control->shoot_RC->rc.s[1]) && switch_is_down(shoot_control->shoot_RC->rc.s[0]))
					shoot.dirver_give_current=0;

    }
    else if(shoot_control->shoot_mode == shoot_fire)
    {
        shoot_limit(shoot_control);
        shoot_control->friction1_speed_pid.set = -fri_spdset;
        shoot_control->friction2_speed_pid.set = fri_spdset;

        if(abs(shoot_control->motor_friction1.shoot_motor_measure->speed_rpm) > 400 && abs(shoot_control->motor_friction2.shoot_motor_measure->speed_rpm) > 400)
        {
						if(shoot_control->shoot_flag ==1&&is_ready
							&&(abs(shoot_control->shoot_RC->rc.ch[4]) == 660 ||shoot_control->shoot_RC->mouse.press_l)&&key_fire==0)////遥控器，鼠标发射,按一次触发一次
						{
							key_fire=1;
							flag_fire=1;
						}
						else if(!(abs(shoot_control->shoot_RC->rc.ch[4]) == 660 || shoot_control->shoot_RC->mouse.press_l))
						{
							key_fire=0;
							flag_fire=0;//发射命令标志位
						}
						else
							key_fire=1;

						if(flag_fire==1/*&& shoot_control->shoot_flag ==1&&shoot_control->remaining_shoot_heat>99*/)        
						{
                shoot_control->friction3_angle_pid.set = shoot_control->motor_friction3.shoot_motor_measure->real_ecd+big_angle*36; 
//								shoot_control->dirver_speed_pid.set = 700; //左击发射，辅助供弹
								flag_fire=0;
          }
            if(!is_ready)
						{
							shoot_control->friction3_angle_pid.set = shoot_control->motor_friction3.shoot_motor_measure->real_ecd+small_angle*36; 
							shoot_control->dirver_speed_pid.set = 600;		//
							flag_done=1;
							shoot_control->block_flag=0;

							if(abs(shoot_control->motor_dirver.dirver_motor_measure->speed_rpm)<100.0f)
							{
								ii++;
								if(ii>1000&&ii<5000)
									shoot_control->block_flag=1;
								else if(ii>10000)	ii=0;
							}
						}
						else
            {
							ii=0;
							shoot_control->block_flag=0;
							shoot_control->dirver_speed_pid.set = 0;
							if(flag_done==1)
							{
								shoot_control->friction3_angle_pid.set = shoot_control->motor_friction3.shoot_motor_measure->real_ecd;
								flag_done=0;
							}
            }
					}
       
        else
            shoot_control->dirver_speed_pid.set = 0;

        shoot_give_current(shoot_control);
    }
}

extern Encoder friction_left,friction_right,friction_up;
static void shoot_give_current(Shoot_t* shoot_give_current)
{

	 shoot_give_current->shoot_give_current[0] = PID_ca(&shoot_give_current->friction1_speed_pid,
																											friction_left.filter_rate, 
																											shoot_give_current->friction1_speed_pid.set);
    shoot_give_current->shoot_give_current[1] = PID_ca(&shoot_give_current->friction2_speed_pid, 
																											friction_right.filter_rate, 
																											shoot_give_current->friction2_speed_pid.set);
	
	 shoot_give_current->friction3_speed_pid.set=PID_ca(&shoot_give_current->friction3_angle_pid, 
																						shoot_give_current->motor_friction3.shoot_motor_measure->real_ecd ,
																						shoot_give_current->friction3_angle_pid.set);
		shoot_give_current->shoot_give_current[2] = PID_ca(&shoot_give_current->friction3_speed_pid,
																								friction_up.filter_rate, 
																								shoot_give_current->friction3_speed_pid.set);
	
		shoot_give_current->dirver_give_current = PID_ca(&shoot_give_current->dirver_speed_pid, 
																											shoot_give_current->motor_dirver.dirver_motor_measure->speed_rpm, 
																											shoot_give_current->dirver_speed_pid.set);

		if(shoot_give_current->block_flag)
			shoot_give_current->dirver_give_current=0;

}

