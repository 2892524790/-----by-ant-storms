#ifndef __FILTER_CONTROL_H__
#define __FILTER_CONTROL_H__
#include "stm32f4xx.h" 
#include "IMUTask.h"

#define Int_Sort    (s16)

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

float invSqrt(float x);
float my_sqrt(float number);
void Acce_Correct_Filter(float x,float y,float z);
void Gyro_Correct_Filter(float x,float y,float z);
float LPButterworth(short curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
float GildeAverageValueFilter_MAG(float NewValue,float *Data);
float constrain(float value, const float min_val, const float max_val);
matrix_3X3_d Atti2Mat_d(atti_d_t atti);
matrix_3X3_d Mat_Trans_d(matrix_3X3_d R);
quater_d_t Mat2Quat(matrix_3X3_d Mat);

#endif


