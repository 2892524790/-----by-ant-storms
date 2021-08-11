/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  * @note       该文件不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "CAN_Receive.h"

#include "detect_task.h"

//底盘电机数据读取 //3508
//#define get_motor_measure(ptr, rx_message)                                                     \
//    {                                                                                          \
//        (ptr)->last_ecd = (ptr)->ecd;                                                          \
//        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
//        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
//        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
//        (ptr)->temperate = (rx_message)->Data[6];                                              \
//    }

#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ; /*反转*/                           \
  else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;/*正转 */                    \
			(ptr)->last_ecd = (ptr)->ecd;                                                          \
            (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
			(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
			(ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
			(ptr)->temperate = (rx_message)->Data[6];                                              \
   (ptr)->all_ecd=((ptr)->count*8191+(ptr)->ecd);	\
   (ptr)->real_ecd=(ptr)->all_ecd*360.f/8192;	\
		}

//云台电机数据读取 //6020
#define get_gimbal_GM6020_motor_measuer(ptr, rx_message)                                       \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//云台电机数据读取 //6623
#define get_gimbal_GM6623_motor_measuer(ptr, rx_message)                                       \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->send_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);  \
    }

		void EncoderProcess3508(Encoder_process_t* v ,motor_measure_t *motor)
{
	int32_t temp_sum = 0; 
		v->diff=motor->ecd-motor->last_ecd;
		if(v->diff < -6500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff > 6500)
		{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff- 8192;
		}
		else
		{
			v->ecd_raw_rate = v->diff;
		}
		v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
		if(v->buf_count == RATE_BUF_SIZE)
		{
			v->buf_count = 0;
		}
		//计算速度平均值
		for(uint8_t i = 0;i < RATE_BUF_SIZE; i++)
		{
			temp_sum += v->rate_buf[i];
		}

		v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
}
		//统一处理can接收函数
static void CAN1_hook(CanRxMsg* rx_message);
static void CAN2_hook(CanRxMsg* rx_message);
//声明电机变量
motor_measure_t motor_chassis[4], motor_friction1, motor_friction2, motor_friction3, motor_dirver, motor_yaw, motor_pit,motor_2006;
Encoder_process_t Encoder_friction[2];
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif
//can1中断

Super_power_t Super_power;

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx1_message;

    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}

//can2中断
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg rx2_message;
    if(CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}


//发送云台控制命令，其中rev为保留字节
void CAN_CMD_GIMBAL(int16_t pitch, int16_t yaw, int16_t driver, int16_t rev)
{
    //can2
    CanTxMsg GIMBAL1_TxMessage;

    GIMBAL1_TxMessage.StdId = 0x1ff;
    GIMBAL1_TxMessage.IDE = CAN_ID_STD;
    GIMBAL1_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL1_TxMessage.DLC = 0x04;
    GIMBAL1_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL1_TxMessage.Data[1] = yaw;
    GIMBAL1_TxMessage.Data[2] = (driver >> 8);
    GIMBAL1_TxMessage.Data[3] = driver;
    CAN_Transmit(CAN1,  &GIMBAL1_TxMessage);
    //can1 205
    CanTxMsg GIMBAL2_TxMessage;
    GIMBAL2_TxMessage.StdId = 0x1ff;
    GIMBAL2_TxMessage.IDE = CAN_ID_STD;
    GIMBAL2_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL2_TxMessage.DLC = 0x02;
    GIMBAL2_TxMessage.Data[0] = (pitch >> 8);
    GIMBAL2_TxMessage.Data[1] = pitch;

    CAN_Transmit(CAN2,  &GIMBAL2_TxMessage);
}

void CAN_CMD_SHOOT(int16_t deiver, int16_t shoot1, int16_t shoot2, int16_t shoot3)
{
    CanTxMsg SHOOT_TxMessage;
    SHOOT_TxMessage.StdId = 0X200;
    SHOOT_TxMessage.IDE = CAN_ID_STD;
    SHOOT_TxMessage.RTR = CAN_RTR_DATA;
    SHOOT_TxMessage.DLC = 0x06;
//    SHOOT_TxMessage.Data[0] = (deiver >> 8);
//    SHOOT_TxMessage.Data[1] = deiver;
    SHOOT_TxMessage.Data[0] = (shoot1 >> 8);
    SHOOT_TxMessage.Data[1] = shoot1;
    SHOOT_TxMessage.Data[2] = (shoot2 >> 8);
    SHOOT_TxMessage.Data[3] = shoot2;
    SHOOT_TxMessage.Data[4] = (shoot3 >> 8);
    SHOOT_TxMessage.Data[5] = shoot3;
    CAN_Transmit(CAN2,  &SHOOT_TxMessage);
}

//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN1, &TxMessage);
}

//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CAN1, &TxMessage);
}


//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t* get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t* get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;//！电机Pitch接收的RxMessage的Data数据传给云台
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t* get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//返回摩擦轮电机变量地址，通过指针方式获得原始数据
const motor_measure_t* get_Friction1__Motor_Measure_Point(void)
{
    return &motor_friction1;//！拨弹电机接收的RxMessage的Data数据传给拨弹
}
//返回摩擦轮电机变量地址，通过指针方式获得原始数据
const motor_measure_t* get_Friction2__Motor_Measure_Point(void)
{
    return &motor_friction2;//！拨弹电机接收的RxMessage的Data数据传给拨弹
}
//返回摩擦轮电机变量地址，通过指针方式获得原始数据
const motor_measure_t* get_Dirver__Motor_Measure_Point(void)
{
    return &motor_dirver;//！拨弹电机接收的RxMessage的Data数据传给拨弹
}
//返回小摩擦轮电机变量地址，通过指针方式获得原始数据
const motor_measure_t* get_Friction3__Motor_Measure_Point(void)
{
    return &motor_friction3;//！拨弹电机接收的RxMessage的Data数据传给拨弹
}
const motor_measure_t* get_2006__Motor_Measure_Point(void)
{
    return &motor_2006;//！拨弹电机接收的RxMessage的Data数据传给拨弹
}

static uint32_t can_count=0;
Encoder friction_left,friction_right,friction_up;
//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN2_hook(CanRxMsg* rx_message)
{
    switch(rx_message->StdId)
    {

    case CAN_PIT_MOTOR_ID:  //205
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_pit, rx_message);
        	DetectHook(PitchGimbalMotorTOE);


    }
    break;
    case CAN_FRICTION1_ID://0x201
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_friction1, rx_message);
//					EncoderProcess3508(&Encoder_friction[0] ,&motor_friction1);
			(can_count<=50)? can_count++,GetEncoderBias(&friction_left,rx_message):EncoderProcess(&friction_left,rx_message);

        //记录时间
             DetectHook(frictionmotorLTOE);

    }
    break;
    case CAN_FRICTION2_ID://0x202
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_friction2, rx_message);
//					EncoderProcess3508(&Encoder_friction[1] ,&motor_friction2);
			(can_count<=50)? can_count++,GetEncoderBias(&friction_right,rx_message):EncoderProcess(&friction_right,rx_message);


        //记录时间
           DetectHook(frictionmotorRTOE);

    }
    break;
    case CAN_FRICTION3_ID://0x203
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_friction3, rx_message);
				(can_count<=50)? can_count++,GetEncoderBias(&friction_up,rx_message):EncoderProcess(&friction_up,rx_message);

        //记录时间
      DetectHook(frictionmotorATOE);

    }
    break;

    default:
    {
        break;
    }
    }
}

static void CAN1_hook(CanRxMsg* rx_message)
{
    switch(rx_message->StdId)
    {
    case CAN_YAW_MOTOR_ID: //205
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_yaw, rx_message);
         DetectHook(YawGimbalMotorTOE);

    }
    break;
    case CAN_DIRVER_ID:  	//206
    {
        //处理电机数据宏函数
        get_motor_measure(&motor_dirver, rx_message);
        //记录时间
          DetectHook(TriggerMotorTOE);

    }
    break;	
//		case 0x207:  	//206
//    {
//        //处理电机数据宏函数
//        get_motor_measure(&motor_2006, rx_message);
//        //记录时间
//          DetectHook(M2006MotorTOE);

//    }
//    break;
    case 0x300:
    {
        Super_power.volt = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
        Super_power.current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
        Super_power.input_volt = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
    }
    break;
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = rx_message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        get_motor_measure(&motor_chassis[i], rx_message);
        //记录时间
          DetectHook(ChassisMotor1TOE + i);
    }
    break;
    default:
    {
        break;
    }
    }

}

void CAN_CMD_SUPERPOWER(int16_t power, int16_t i)
{
    CanTxMsg SendCanTxMsg;

    SendCanTxMsg.StdId = 0x222;
    SendCanTxMsg.IDE = CAN_ID_STD;
    SendCanTxMsg.RTR = CAN_RTR_DATA;
    SendCanTxMsg.DLC = 0x02;
    SendCanTxMsg.Data[0] = power;
    SendCanTxMsg.Data[1] = i;
    CAN_Transmit(CAN1, &SendCanTxMsg);
}

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
    v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差
    v->ecd_value = v->ecd_bias;
    v->raw_value = v->ecd_bias;
    v->temp_count++;
}


void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
    int i=0;
    int32_t temp_sum = 0;

    v->last_raw_value = v->raw_value;
    v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
    v->diff = v->raw_value - v->last_raw_value;

    if(v->diff < -6500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff>6500)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff- 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }

    //计算得到连续的编码器输出值
    v->ecd_value = v->raw_value + v->round_cnt * 8192;
    //计算得到角度值，范围正负无穷大
    v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if(v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }
    //计算速度平均值
    for(i = 0; i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }
    v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
}

