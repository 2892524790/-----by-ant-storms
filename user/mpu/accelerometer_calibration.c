#include "accelerometer_calibration.h"
#include "arm_math.h"
#include "IMUTask.h"
#include "bsp_flash.h"
#define mat         arm_matrix_instance_f32
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

#define CALIBRATION_ID  Calibrateion_id

/*
各ID与六面的关系
ID     x       y       z
0      天      西      南
1      地      东      南
2      东      天      南
3      西      地      南
4      东      北      天
5      南      西      地
*/

#define OFFICE_X  MPU6500_Offest_data.Accel_X
#define OFFICE_Y  MPU6500_Offest_data.Accel_Y
#define OFFICE_Z  MPU6500_Offest_data.Accel_Z

int Calibrateion_id = 9;

float	Ahat_data[3][6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
                        };

float K_m_data[3][4] = {0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0f
                       };

float ahat_T_data[6][4] = {GRAVITY_MSS, 0.0f, 0.0f, 1.0f,
                           -GRAVITY_MSS, 0.0f, 0.0f, 1.0f,
                           0.0f, GRAVITY_MSS, 0.0f, 1.0f,
                           0.0f, -GRAVITY_MSS, 0.0f, 1.0f,
                           0.0f, 0.0f, GRAVITY_MSS, 1.0f,
                           0.0f, 0.0f, -GRAVITY_MSS, 1.0f
                           };

float ahat_data[4][6] = {GRAVITY_MSS, -GRAVITY_MSS, 0, 0, 0, 0,
                         0, 0, GRAVITY_MSS, -GRAVITY_MSS, 0, 0,
                         0, 0, 0, 0, GRAVITY_MSS, -GRAVITY_MSS,
                         1.0, 1.0, 1.0, 1.0, 1.0, 1.0
                        };
float uMat_data[4][4], uMat_mius_data[4][4], P_data[6][4];
float K_ca_data[9], a_data[3], K_ca_mines_data[9];	 //实际上用来计算最终结果的矩阵
mat ahat_T, ahat, Ahat, K_m;
mat uMat, uMat_mius, P;
mat K_ca, K_ca_mines, a_m;

void accel_mat_init(void)
{
    mat_init(&ahat, 4, 6, (float*)ahat_data);
    mat_init(&ahat_T, 6, 4, (float*)ahat_T_data);
    mat_init(&K_m, 3, 4, (float*)K_m_data);
    mat_init(&uMat, 4, 4, (float*)uMat_data);
    mat_init(&uMat_mius, 4, 4, (float*)uMat_mius_data);
    mat_init(&P, 6, 4, (float*)P_data);
    mat_init(&Ahat, 3, 6, (float*)Ahat_data);

    mat_init(&K_ca, 3, 3, (float*)K_ca_data);
    mat_init(&a_m, 3, 3, (float*)a_data);
    mat_init(&K_ca_mines, 3, 3, (float*)K_ca_mines_data);
    FlashReadK((u8*)(K_m.pData));
    K_ca.pData[0] = K_m.pData[0];
    K_ca.pData[1] = K_m.pData[1];
    K_ca.pData[2] = K_m.pData[2];
    a_m.pData[0] = K_m.pData[3];
    K_ca.pData[3] = K_m.pData[4];
    K_ca.pData[4] = K_m.pData[5];
    K_ca.pData[5] = K_m.pData[6];
    a_m.pData[1] = K_m.pData[7];
    K_ca.pData[6] = K_m.pData[8];
    K_ca.pData[7] = K_m.pData[9];
    K_ca.pData[8] = K_m.pData[10];
    a_m.pData[2] = K_m.pData[11];
    mat_inv(&K_ca, &K_ca_mines);
}

//计算校准矩阵K
void cali_accel_para(void)
{
    mat_mult(&ahat, &ahat_T, &uMat);
    mat_inv(&uMat, &uMat_mius);
    mat_mult(&ahat_T, &uMat_mius, &P);
    mat_mult(&Ahat, &P, &K_m);

    K_ca.pData[0] = K_m.pData[0];
    K_ca.pData[1] = K_m.pData[1];
    K_ca.pData[2] = K_m.pData[2];
    a_m.pData[0] = K_m.pData[3];
    K_ca.pData[3] = K_m.pData[4];
    K_ca.pData[4] = K_m.pData[5];
    K_ca.pData[5] = K_m.pData[6];
    a_m.pData[1] = K_m.pData[7];
    K_ca.pData[6] = K_m.pData[8];
    K_ca.pData[7] = K_m.pData[9];
    K_ca.pData[8] = K_m.pData[10];
    a_m.pData[2] = K_m.pData[11];
    mat_inv(&K_ca, &K_ca_mines);
    FlashWriteK((u8*)(K_m.pData));
}
//更新六面数据采集值
int accel_data_update(void)
{
    int now_num = 6;
    while(CALIBRATION_ID <= 7)
    {
        if((now_num != CALIBRATION_ID) && (now_num <= 6))
        {
            Accel_calibration_get();
            Ahat_data[0][CALIBRATION_ID] = OFFICE_X;
            Ahat_data[1][CALIBRATION_ID] = OFFICE_Y;
            Ahat_data[2][CALIBRATION_ID] = OFFICE_Z;
            now_num = CALIBRATION_ID;
        }
        if(CALIBRATION_ID == 7)
        {
            cali_accel_para();
            CALIBRATION_ID = 8;
            return 0;
        }
    }
    return 0;
}



//加速度计校准函数
void cali_fenction(void)
{
    accel_data_update();

}

//得到计算实际加速度相关的矩阵,并计算校准后的加速度值
void caculate_mat(float* ax, float* ay, float* az)
{
    float temp1_data[3] = {*ax - a_m.pData[0], *ay - a_m.pData[1], *az - a_m.pData[2]}, temp2_data[3];
    mat temp1, temp2;
    mat_init(&temp1, 3, 1, (float*)temp1_data);
    mat_init(&temp2, 3, 1, (float*)temp2_data);
    mat_mult(&K_ca_mines, &temp1, &temp2);
    *ax = temp2.pData[0];
    *ay = temp2.pData[1];
    *az = temp2.pData[2];
}
