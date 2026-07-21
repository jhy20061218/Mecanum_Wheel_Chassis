//
// Created by 34285 on 2026/6/29.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ROTATION_MATRIX_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ROTATION_MATRIX_H

#include <stdlib.h>
#include <math.h>

//IMU物理量数据结构体(9轴数据，如果是6轴，不用管磁力计)
typedef struct
{
    //加速度计数据
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    //陀螺仪数据
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    //磁力计数据
    float Mag_X;
    float Mag_Y;
    float Mag_Z;
    //欧拉角数据
    float Roll;
    float Pitch;
    float Yaw;
}Struct_IMU_Data;

// 定义矩阵结构体
typedef struct
{
    int rows;
    int columns;
    float *data; // 使用一维数组存储连续数据
}Struct_Matrix;

/**
 * @brief 云台电机编码器数据结构体
 */
typedef struct
{
    float Yaw;
    float Pitch;
    // float Roll; // 步兵机器人云台一般没有Roll轴，如果有请取消注释
} Struct_Gimbal_Encoder_Data;

Struct_Matrix Create_Matrix(int rows, int cols);
void Free_Matrix(Struct_Matrix *matrix);

Struct_Matrix Calculate_2D_Rotation_Matrix(float alpha);
Struct_Matrix Calculate_3D_Yaw_Matrix(float alpha);
Struct_Matrix Calculate_3D_Pitch_Matrix(float beta);
Struct_Matrix Calculate_3D_Roll_Matrix(float theta);

Struct_Matrix Calculate_Extrinsic_Rotation_Matrix(float alpha, float beta, float theta);
Struct_Matrix Calculate_Intrinsic_Rotation_Matrix(float alpha, float beta, float theta);

Struct_Matrix Get_Chassis_Rotation_Matrix(Struct_IMU_Data* imu_data);
float Get_Slope_Pitch(Struct_IMU_Data* imu_data);
float Get_Slope_Yaw(Struct_IMU_Data* imu_data);
float Get_Chassis_Yaw_Relative_To_Slope(Struct_IMU_Data* imu_data);

Struct_Matrix Multiply_Matrix(Struct_Matrix A, Struct_Matrix B);
Struct_Matrix Get_Gimbal_Rotation_Matrix(Struct_IMU_Data* imu_data, Struct_Gimbal_Encoder_Data* encoder_data);


#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ROTATION_MATRIX_H
