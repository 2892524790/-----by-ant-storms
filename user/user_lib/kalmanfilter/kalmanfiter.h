#ifndef __KALMANFITER_H
#define __KALMANFITER_H
#include "stm32f4xx.h"
#include "arm_math.h"

#define mat arm_matrix_instance_f32
#define mat_64 arm_matrix_instance_f64
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

//�������˲����Խṹ��
typedef struct
{
  long x_PC;
  long x_EX;
  long y_PC;
  long y_EX;
  long z_PC;
  long z_EX;
} kalmant_text;

extern kalman_filter_init_t KalmanInit;
extern kalman_filter_t KalmanMat;

extern kalmant_text KalManText;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
#endif
