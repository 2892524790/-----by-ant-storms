#include "Filter_Control.h"
#include "fdacoefs.h"


Butter_BufferData Acce_BufferData[3];
Filter_t Acce_Fiter_t[3];
Filter_t Gyro_Fiter_t[3];
//-----Butterworth变量 Matlab fdatool设计-----//
//设计方法见博客：http://blog.csdn.net/u011992534/article/details/73743955
/*
Butter_Parameter Butter_80HZ_Parameter_Acce={
  //200hz---80hz
1,     1.14298050254,   0.4128015980962,
0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter Butter_60HZ_Parameter_Acce={
  //200hz---60hz
1,   0.3695273773512,   0.1958157126558,
0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter Butter_51HZ_Parameter_Acce={
  //200hz---51hz
1,  0.03680751639284,   0.1718123812701,
0.3021549744157,   0.6043099488315,   0.3021549744157,
};

Butter_Parameter Butter_30HZ_Parameter_Acce={
  //200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Butter_20HZ_Parameter_Acce={
  //200hz---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};
Butter_Parameter Butter_15HZ_Parameter_Acce={
  //200hz---15hz
  1,   -1.348967745253,   0.5139818942197,
  0.04125353724172,  0.08250707448344,  0.04125353724172
};

Butter_Parameter Butter_10HZ_Parameter_Acce={
  //200hz---10hz
  1,   -1.561018075801,   0.6413515380576,
  0.02008336556421,  0.04016673112842,  0.02008336556421};
Butter_Parameter Butter_5HZ_Parameter_Acce={
  //200hz---5hz
  1,   -1.778631777825,   0.8008026466657,
  0.005542717210281,  0.01108543442056, 0.005542717210281
};

Butter_Parameter Butter_2HZ_Parameter_Acce={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};
*/

Butter_Parameter Butter_30HZ_Parameter_Acce =
{
//200hz---30hz
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166
};
void Acce_Correct_Filter(float x, float y, float z)
{



//		Acce_Fiter_t[0].raw_value = x;
//	  Acce_Fiter_t[1].raw_value = y;
//	  Acce_Fiter_t[2].raw_value = z;
//	 MPU9250_Real_Data.Accel_X=Chebyshev50HzLPF(&Acce_Fiter_t[0]);
//   MPU9250_Real_Data.Accel_Y=Chebyshev50HzLPF(&Acce_Fiter_t[1]);
//   MPU9250_Real_Data.Accel_Z=Chebyshev50HzLPF(&Acce_Fiter_t[2]);

//   MPU9250_Real_Data.Accel_X=Int_Sort(LPButterworth( x,  &Acce_BufferData[0],&Butter_30HZ_Parameter_Acce));
//   MPU9250_Real_Data.Accel_Y=Int_Sort(LPButterworth( y,  &Acce_BufferData[1],&Butter_30HZ_Parameter_Acce));
//   MPU9250_Real_Data.Accel_Z=Int_Sort(LPButterworth( z,  &Acce_BufferData[2],&Butter_30HZ_Parameter_Acce));
    MPU9250_Real_Data.Accel_X = x;
    MPU9250_Real_Data.Accel_Y = y;
    MPU9250_Real_Data.Accel_Z = z;


}

Butter_BufferData Gyro_BufferData[3];
Butter_Parameter Butter_51HZ_Parameter_Gyro =
{
    //200hz---51hz
    1,  0.03680751639284,   0.1718123812701,
    0.3021549744157,   0.6043099488315,   0.3021549744157
};

void Gyro_Correct_Filter(float x, float y, float z)
{
//   MPU9250_Real_Data.Gyro_X=Int_Sort(LPButterworth( x,  &Gyro_BufferData[0],&Butter_51HZ_Parameter_Gyro));
//   MPU9250_Real_Data.Gyro_Y=Int_Sort(LPButterworth( y,  &Gyro_BufferData[1],&Butter_51HZ_Parameter_Gyro));
//   MPU9250_Real_Data.Gyro_Z=Int_Sort(LPButterworth( z,  &Gyro_BufferData[2],&Butter_51HZ_Parameter_Gyro));

    Gyro_Fiter_t[0].raw_value = x;
    Gyro_Fiter_t[1].raw_value = y;
    Gyro_Fiter_t[2].raw_value = z;
    MPU9250_Real_Data.Gyro_X = Chebyshev50HzLPF(&Gyro_Fiter_t[0]);
    MPU9250_Real_Data.Gyro_Y = Chebyshev50HzLPF(&Gyro_Fiter_t[1]);
    MPU9250_Real_Data.Gyro_Z = Chebyshev50HzLPF(&Gyro_Fiter_t[2]);
//	   MPU9250_Real_Data.Gyro_X=x;
//		 MPU9250_Real_Data.Gyro_Y=y;
//		 MPU9250_Real_Data.Gyro_Z=z;

}

/*************************************************
函数名:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	short curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(short curr_input, Butter_BufferData* Buffer, Butter_Parameter* Parameter)
{
    Buffer->Input_Butter[2] = curr_input;
    /* Butterworth滤波 */
    Buffer->Output_Butter[2] =	Parameter->b[0] * Buffer->Input_Butter[2]
                                + Parameter->b[1] * Buffer->Input_Butter[1]
                                + Parameter->b[2] * Buffer->Input_Butter[0]
                                - Parameter->a[1] * Buffer->Output_Butter[1]
                                - Parameter->a[2] * Buffer->Output_Butter[0];

    /* x(n) 序列保存 */
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    /* y(n) 序列保存 */
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
    return Buffer->Output_Butter[2];
}

float GildeAverageValueFilter_MAG(float NewValue, float* Data)
{
    float max, min;
    float sum;
    unsigned char i;
    Data[0] = NewValue;
    max = Data[0];
    min = Data[0];
    sum = Data[0];

    for(i = 9; i != 0; i--)
    {
        if(Data[i] > max) max = Data[i];
        else if(Data[i] < min) min = Data[i];
        sum += Data[i];
        Data[i] = Data[i - 1];
    }
    sum = sum - max - min;
    sum = sum / 8;
    return(sum);
}
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}
//快速平方根算法
float my_sqrt(float number)
{
    long i;
    float x, y;
    const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (long*) &y;
    i = 0x5f3759df - (i >> 1);

    y = * (float*) &i;
    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));
    return number * y;
}

float constrain(float value, const float min_val, const float max_val)
{
    if(value >= max_val)  value = max_val;
    if(value <= min_val)  value = min_val;
    return value;
}

matrix_3X3_d Atti2Mat_d(atti_d_t atti)
{
    matrix_3X3_d R;
    double psi, phy, sita;
    psi = atti.yaw * DEG2RAD;
    sita = atti.pitch * DEG2RAD;
    phy = atti.roll * DEG2RAD;
    R.r_11 = cos(sita) * cos(psi);
    R.r_12 = sin(phy) * sin(sita) * cos(psi) - cos(phy) * sin(psi);
    R.r_13 = cos(phy) * sin(sita) * cos(psi) + sin(phy) * sin(psi);
    R.r_21 = cos(sita) * sin(psi);
    R.r_22 = sin(phy) * sin(sita) * sin(psi) + cos(phy) * cos(psi);
    R.r_23 = cos(phy) * sin(sita) * sin(psi) - sin(phy) * cos(psi);
    R.r_31 = -sin(sita);
    R.r_32 = sin(phy) * cos(sita);
    R.r_33 = cos(phy) * cos(sita);
    return R;
}
matrix_3X3_d Mat_Trans_d(matrix_3X3_d R)
{
    matrix_3X3_d R1;
    R1.r_11 = R.r_11;
    R1.r_12 = R.r_21;
    R1.r_13 = R.r_31;
    R1.r_21 = R.r_12;
    R1.r_22 = R.r_22;
    R1.r_23 = R.r_32;
    R1.r_31 = R.r_13;
    R1.r_32 = R.r_23;
    R1.r_33 = R.r_33;
    return R1;
}

quater_d_t Mat2Quat(matrix_3X3_d Mat)
{
    double temp_d;
    quater_d_t Quat;
    if(Mat.r_22 > -Mat.r_33)
    {
        if(Mat.r_22 > Mat.r_33)
            if(Mat.r_11 > -Mat.r_33)
            {
                temp_d = 2.0 * sqrt(1 + Mat.r_11 + Mat.r_22 + Mat.r_33);
                Quat.q0 = temp_d / 4.0;
                Quat.q1 = (Mat.r_23 - Mat.r_32) / temp_d;
                Quat.q2 = (Mat.r_31 - Mat.r_13) / temp_d;
                Quat.q3 = (Mat.r_12 - Mat.r_21) / temp_d;
            }
            else
            {
                temp_d  = 2.0 * sqrt(1 - Mat.r_11 + Mat.r_22 - Mat.r_33);
                Quat.q0 = (Mat.r_13 - Mat.r_31) / temp_d;
                Quat.q1 = -(Mat.r_21 + Mat.r_12) / temp_d;
                Quat.q2 = -temp_d / 4.0;
                Quat.q3 = -(Mat.r_32 + Mat.r_23) / temp_d;
            }
        else
        {
            if(Mat.r_11 > -Mat.r_22)
            {
                temp_d = 2.0 * sqrt(1 + Mat.r_11 + Mat.r_22 + Mat.r_33);
                Quat.q0 = temp_d / 4.0;
                Quat.q1 = (Mat.r_23 - Mat.r_32) / temp_d;
                Quat.q2 = (Mat.r_31 - Mat.r_13) / temp_d;
                Quat.q3 = (Mat.r_12 - Mat.r_21) / temp_d;
            }
            else
            {
                temp_d  = 2.0 * sqrt(1 - Mat.r_11 - Mat.r_22 + Mat.r_33);
                Quat.q0 = (Mat.r_21 - Mat.r_12) / temp_d;
                Quat.q1 = -(Mat.r_13 + Mat.r_31) / temp_d;
                Quat.q2 = -(Mat.r_32 + Mat.r_23) / temp_d;
                Quat.q3 = -temp_d / 4.0;

            }
        }
    }
    else
    {
        if(Mat.r_22 > Mat.r_33)
        {
            if(Mat.r_11 > Mat.r_22)
            {
                temp_d  = 2.0 * sqrt(1 + Mat.r_11 - Mat.r_22 - Mat.r_33);
                Quat.q0 = (Mat.r_32 - Mat.r_23) / temp_d;
                Quat.q1 = -temp_d / 4.0;
                Quat.q2 = -(Mat.r_21 + Mat.r_12) / temp_d;
                Quat.q3 = -(Mat.r_13 + Mat.r_31) / temp_d;
            }
            else
            {
                temp_d  = 2.0 * sqrt(1 - Mat.r_11 + Mat.r_22 - Mat.r_33);
                Quat.q0 = (Mat.r_13 - Mat.r_31) / temp_d;
                Quat.q1 = -(Mat.r_21 + Mat.r_12) / temp_d;
                Quat.q2 = -temp_d / 4.0;
                Quat.q3 = -(Mat.r_32 + Mat.r_23) / temp_d;
            }
        }
        else
        {
            if(Mat.r_11 > Mat.r_33)
            {
                temp_d  = 2.0 * sqrt(1 + Mat.r_11 - Mat.r_22 - Mat.r_33);
                Quat.q0 = (Mat.r_32 - Mat.r_23) / temp_d;
                Quat.q1 = -temp_d / 4.0;
                Quat.q2 = -(Mat.r_21 + Mat.r_12) / temp_d;
                Quat.q3 = -(Mat.r_13 + Mat.r_31) / temp_d;
            }
            else
            {
                temp_d  = 2.0 * sqrt(1 - Mat.r_11 - Mat.r_22 + Mat.r_33);
                Quat.q0 = (Mat.r_21 - Mat.r_12) / temp_d;
                Quat.q1 = -(Mat.r_13 + Mat.r_31) / temp_d;
                Quat.q2 = -(Mat.r_32 + Mat.r_23) / temp_d;
                Quat.q3 = -temp_d / 4.0;
            }
        }
    }
    if(Quat.q0 < 0)
    {
        Quat.q0 = -Quat.q0;
        Quat.q1 = -Quat.q1;
        Quat.q2 = -Quat.q2;
        Quat.q3 = -Quat.q3;
    }
    return Quat;
}

