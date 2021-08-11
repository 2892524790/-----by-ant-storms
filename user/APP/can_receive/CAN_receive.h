/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"
#define  RATE_BUF_SIZE 5

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
		CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,///////CAN2
    CAN_PIT_MOTOR_ID = 0x205,///////CAN1
	
		CAN_SHOOT_ALL_ID = 0X200,
		CAN_FRICTION1_ID = 0x201,
		CAN_FRICTION2_ID = 0x202,
		CAN_FRICTION3_ID=	 0X203,
		CAN_DIRVER_ID = 0x206,///////CAN2

} can_msg_id_e;

typedef struct
{
	int32_t diff;	
	int32_t round_cnt;
	int32_t ecd_raw_rate;
	int32_t rate_buf[RATE_BUF_SIZE]; 	//buf��for filter
	uint8_t buf_count;					//�˲�����buf��
	int32_t filter_rate;				//�ٶ�
} Encoder_process_t;
//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;

    int16_t send_current; //6623  ����
    uint8_t temperate;
    int16_t last_ecd;
    int16_t count;
    fp32 all_ecd;
	fp32 real_ecd;
} motor_measure_t;

typedef struct
{
    uint16_t 	volt;
    uint16_t	current;
    uint16_t	input_volt;
} Super_power_t;

typedef struct {
    int32_t raw_value;   								//���������������ԭʼֵ
    int32_t last_raw_value;							//��һ�εı�����ԭʼֵ
    int32_t ecd_value;                  //��������������ı�����ֵ
    int32_t diff;										    //���α�����֮��Ĳ�ֵ
    int32_t temp_count;                 //������
    uint8_t buf_count;								  //�˲�����buf��
    int32_t ecd_bias;									  //��ʼ������ֵ
    int32_t ecd_raw_rate;								//ͨ������������õ����ٶ�ԭʼֵ
    int32_t rate_buf[RATE_BUF_SIZE];	  //f��for filter
    int32_t round_cnt;									//Ȧ��
    int32_t filter_rate;								//�ٶ�
    float ecd_angle;								  	//�Ƕ�
} Encoder;




extern void CAN_CMD_CHASSIS_RESET_ID(void);

//���͵��̵����������
void CAN_CMD_SHOOT(int16_t deiver, int16_t shoot1, int16_t shoot2, int16_t shoot3);
//extern  Angular_Handle  Angular_Handler;
//������̨�����������revΪ�����ֽ�
void CAN_CMD_GIMBAL(int16_t pitch, int16_t yaw, int16_t driver, int16_t rev);
//���͵��̵����������
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t* get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t* get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t* get_Trigger_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t* get_Chassis_Motor_Measure_Point(uint8_t i);
//����Ħ���ֵ������
extern const motor_measure_t* get_Friction2__Motor_Measure_Point(void);
//����Ħ���ֵ������
extern const motor_measure_t* get_Friction1__Motor_Measure_Point(void);
extern const motor_measure_t* get_Dirver__Motor_Measure_Point(void);
extern const motor_measure_t* get_Friction3__Motor_Measure_Point(void);
extern const motor_measure_t* get_2006__Motor_Measure_Point(void);

//������������ģ��
extern Super_power_t Super_power;
extern void CAN_CMD_SUPERPOWER(int16_t power, int16_t i);

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
