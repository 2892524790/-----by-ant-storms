#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

#include "main.h"
enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};

typedef struct FFC_t
{
  float A;
  float B;
  float rin;
  float lastRin;
  float perrRin;
  float out;
} FFC;

typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;   //input max err;
  float err_deadband;    //error  deadband;
  float output_deadband; //output deadband;

  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;
  float proportion; //占输出总比
  float final_out;  //最终输出到电机的设定值
  s8 out_sign;      //输入时的正负标志
  void (*f_param_init)(struct pid_t *pid,
                       uint32_t pid_mode,
                       uint32_t max_output,
                       uint32_t inte_limit,
                       float p,
                       float i,
                       float d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);

} pid_t;

//#define Inter_Max   500
typedef struct PID_PARAMETER
{
  float ref;
  float fdb;
  float Kp;
  float Ki;
  float Kd;
  float error_now;   //误差
  float error_last;  //上一次误差
  float error_rate;  //误差变化率
  float error_inter; //误差积分
  float Pout;
  float Iout;
  float Dout;
  float error_prev; //上上次误差
  float pid_out;
  float proportion; //占输出总比
  float final_out;  //最终输出到电机的设定值
  s8 out_sign;      //输入时的正负标志

} PID;

typedef struct te
{
  int PID_ref;
  int PID_fdb;
  int PID_errol;
} pidtest;

void PID_Set(PID *motor_type, float Kp, float Ki, float Kd);

void PID_Control(PID *motor_type);
void PID_Reset(PID *motor_type);
void PID_Speed_Calc(PID *motor_type);
void PID_Control_Motor(PID *motor_type);

void FeedforwardController(FFC *vFFC);
void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float fdb, float ref);
//float pid_calc_err_limit(pid_t *pid, float fdb, float ref);
//void pid_reset(pid_t *pid, float kp, float ki, float kd);
//extern pid_t pid_pit;
//extern pid_t pid_yaw;
//extern pid_t pid_pit_spd;
//extern pid_t pid_yaw_spd;
//extern pid_t pid_spd[4];
//extern FFC  FFC_Pith;
//extern pid_t pid_chassis_angle;
//extern pid_t pid_trigger;
//extern pid_t pid_trigger_spd;
//extern pid_t pid_imu_tmp;
//extern pid_t pid_shoot;
//extern pid_t pid_driver_spd ;
//extern pid_t pid_friction_l ;
//extern pid_t pid_friction_r ;
#endif
