#include "PID_Control.h"

/***********位置pid***********************/

#define Inter_Max 2000

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */

void abs_limit(float* a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd)
{

    pid->integral_limit = intergral_limit;
    pid->max_out = maxout;
    pid->pid_mode = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
//void pid_reset(pid_t *pid, float kp, float ki, float kd)
//{
//  pid->p = kp;
//  pid->i = ki;
//  pid->d = kd;

//  //  pid->pout = 0;
//  //  pid->iout = 0;
//  //  pid->dout = 0;
//  //  pid->out  = 0;
//}
/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    if((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
        return 0;

    if(pid->pid_mode == POSITION_PID)  //position PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }
    else if(pid->pid_mode == DELTA_PID)  //delta PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->out += pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];

    if((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
        return 0;
    else
        return pid->out;
}

/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
    pid->f_param_init = pid_param_init;
    //pid->f_pid_reset = pid_reset;

    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
    pid->f_pid_reset(pid, kp, ki, kd);
}

//pid_t pid_yaw = {0};
//pid_t pid_pit = {0};
//pid_t pid_yaw_spd = {0};
//pid_t pid_pit_spd = {0};
//pid_t pid_spd[4] = {0};
//pid_t pid_chassis_angle = {0};
//pid_t pid_trigger = {0};
//pid_t pid_trigger_spd = {0};
//pid_t pid_imu_tmp = {0};
//pid_t pid_shoot = {0};
//pid_t pid_driver_spd = {0};
//pid_t pid_friction_l = {0};
//pid_t pid_friction_r = {0};

void PID_Control(PID* motor_type)
{
    //  float error_position;
    motor_type->error_last = motor_type->error_now;
    motor_type->error_now = motor_type->ref - motor_type->fdb;
    motor_type->error_rate = motor_type->error_now - motor_type->error_last;
    motor_type->error_inter += motor_type->error_now;
    // limit intergration of pid
    if(motor_type->error_inter > Inter_Max)
        motor_type->error_inter = Inter_Max;
    if(motor_type->error_inter < -Inter_Max)
        motor_type->error_inter = -Inter_Max;

    motor_type->Iout = motor_type->Ki * motor_type->error_inter;
    motor_type->Pout = motor_type->Kp * motor_type->error_now;
    motor_type->Dout = motor_type->Kd * motor_type->error_last;

    motor_type->pid_out = (motor_type->Pout + motor_type->Iout + motor_type->Dout);
}
/*********************增量式PID******************************/

void PID_Speed_Calc(PID* motor_type)
{

    if(motor_type->ref < 4 && motor_type->ref > -4)
    {
        motor_type->ref = 0;
    }
    motor_type->error_now = motor_type->ref - motor_type->fdb;

    motor_type->pid_out = motor_type->pid_out + motor_type->Kp * motor_type->error_now //E[k]
                          - motor_type->Ki * motor_type->error_last                    //E[k-1]
                          + motor_type->Kd * motor_type->error_prev;                   //E[k-2]

    motor_type->error_prev = motor_type->error_last;
    motor_type->error_last = motor_type->error_now;
}

void PID_Reset(PID* motor_type)
{
    motor_type->ref = 0;
    motor_type->fdb = 0;
    motor_type->Kp = 0;
    motor_type->Ki = 0;
    motor_type->Kd = 0;
    motor_type->error_last = 0;
    motor_type->error_now = 0;
    motor_type->error_rate = 0;
    motor_type->error_inter = 0;
    motor_type->Iout = 0;
    motor_type->Dout = 0;
    motor_type->Pout = 0;
    motor_type->error_prev = 0;
    motor_type->pid_out = 0;
    motor_type->out_sign = 0;
    motor_type->proportion = 0;
    motor_type->final_out = 0;
}

void PID_Set(PID* motor_type, float Kp, float Ki, float Kd)
{
    motor_type->Kp = Kp;
    motor_type->Ki = Ki;
    motor_type->Kd = Kd;
}
