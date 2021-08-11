#ifndef _IMUTASK_H_
#define _IMUTASK_H_
#include "stm32f4xx.h"
#include "arm_math.h"
#include "BSP_MPU9250_Init.h"
#include "accelerometer_calibration.h"
#define AcceMax_1G      4096
#define GRAVITY_MSS     9.80665f
#define ACCEL_TO_1G     GRAVITY_MSS/AcceMax_1G
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS
#define DEG2RAD         (PI / 180.0f)
#define RAD2DEG         (180.0f / PI)

#define GYRO_CALIBRATION_COFF_2 0.060976f  //2000
#define GYRO_CALIBRATION_COFF   0.030488f    //1000

extern  MPU9250_DATA_t   MPU9250_Real_Data;
extern MPU9250_DATA   MPU9250_Raw_Data;
extern MPU9250_DATA   MPU9250_Offest_Data;

extern Angular_Handle  Angular_Handler;
extern mpu6500_real_data_t mpu6500_real_data;
extern unsigned char spi_rtx[sizeof(mpu6500_real_data_t)];
extern float K[3],B[3];
extern MPU9250_DATA_t MPU6500_Offest_data;

typedef struct
{
	double roll; 
	double pitch;
	double yaw;
}atti_d_t;

typedef struct
{
 float x;
 float y;
}Vector2f;

typedef struct
{
 float x;
 float y;
 float z;
}Vector3f;

typedef struct
{
  Vector3f a;
  Vector3f b;
  Vector3f c;
}Matrix3f;

typedef struct {
    float thx;
    float thy;
    int16_t x;
    int16_t y;
    int16_t z;
    float Angle_Mag;
}AK8963;

typedef struct
{
	double  q0; 
	double 	q1;
	double 	q2;
	double 	q3;
}quater_d_t;

typedef struct
{
	double r_11;
	double r_12;
	double r_13;
	double r_21;
	double r_22;
	double r_23;
	double r_31;
	double r_32;
	double r_33;
}matrix_3X3_d;
 
void IMU_task(void  *pvParameters);
void IMU_Calibration(void);
void ACCEL_Calibration(void);
void GYRO_Filter(void);
void ACCEL_Filter(void);
void Mag_Filter(void);
void AHRSUpdate_GraDes_Delay_Corretion( void);
void DeclineG(void );
void IMU_TempInit(void);
void IMU_TempContrl(void);
void Accel_calibration_get(void);
//·µ»ØÍÓÂÝÒÇÊý¾Ý
extern const Angular_Handle *get_Gyro_Angle_Point(void);

void imu_temp_keep(void);
void mpu_heat_output(uint16_t pwm_pulse);

#endif

