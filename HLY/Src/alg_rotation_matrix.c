//
// Created by 34285 on 2026/6/29.
//
#include "alg_rotation_matrix.h"

/**
 * @brief 创建一个指定行数和列数的矩阵
 *
 * @param rows 行数  
 * @param cols 列数
 * @return Struct_Matrix 创建的矩阵
 */
Struct_Matrix Create_Matrix(int rows, int cols)
{
    Struct_Matrix matrix;
    matrix.rows = rows;
    matrix.columns = cols;
    matrix.data = (float *)calloc(rows * cols, sizeof(float)); //calloc自动初始化为0
    return matrix;
}

/**
 * @brief 释放矩阵的内存
 *
 * @param matrix 要释放的矩阵
 */
void Free_Matrix(Struct_Matrix *matrix)
{
    if (matrix->data != NULL)
    {
        free(matrix->data);
        matrix->data = NULL;
    }
    matrix->rows = 0;
    matrix->columns = 0;
}

/**
 * @brief 二维坐标系的旋转矩阵
 *
 * @param alpha 旋转角度（弧度制）
 * @return Struct_Matrix 2x2的旋转矩阵
 */
Struct_Matrix Calculate_2D_Rotation_Matrix(float alpha)
{
    Struct_Matrix matrix = Create_Matrix(2, 2);

    matrix.data[0 * matrix.columns + 0] = cosf(alpha);
    matrix.data[0 * matrix.columns + 1] = -sinf(alpha);
    matrix.data[1 * matrix.columns + 0] = sinf(alpha);
    matrix.data[1 * matrix.columns + 1] = cosf(alpha);

    return matrix;
}

/**
 * @brief 三维空间下 Z轴/Yaw 旋转矩阵
 *
 * @param alpha Yaw旋转角度（弧度制）
 * @return Struct_Matrix 3x3的旋转矩阵
 */
Struct_Matrix Calculate_3D_Yaw_Matrix(float alpha)
{
    Struct_Matrix matrix = Create_Matrix(3, 3);

    matrix.data[0 * matrix.columns + 0] = cosf(alpha);
    matrix.data[0 * matrix.columns + 1] = -sinf(alpha);
    matrix.data[0 * matrix.columns + 2] = 0;

    matrix.data[1 * matrix.columns + 0] = sinf(alpha);
    matrix.data[1 * matrix.columns + 1] = cosf(alpha);
    matrix.data[1 * matrix.columns + 2] = 0;

    matrix.data[2 * matrix.columns + 0] = 0;
    matrix.data[2 * matrix.columns + 1] = 0;
    matrix.data[2 * matrix.columns + 2] = 1;

    return matrix;
}

/**
 * @brief 三维空间下 Y轴/Pitch 旋转矩阵
 *
 * @param beta Pitch旋转角度（弧度制）
 * @return Struct_Matrix 3x3的旋转矩阵
 */
Struct_Matrix Calculate_3D_Pitch_Matrix(float beta)
{
    Struct_Matrix matrix = Create_Matrix(3, 3);

    matrix.data[0 * matrix.columns + 0] = cosf(beta);
    matrix.data[0 * matrix.columns + 1] = 0;
    matrix.data[0 * matrix.columns + 2] = sinf(beta);

    matrix.data[1 * matrix.columns + 0] = 0;
    matrix.data[1 * matrix.columns + 1] = 1;
    matrix.data[1 * matrix.columns + 2] = 0;

    matrix.data[2 * matrix.columns + 0] = -sinf(beta);
    matrix.data[2 * matrix.columns + 1] = 0;
    matrix.data[2 * matrix.columns + 2] = cosf(beta);

    return matrix;
}

/**
 * @brief 三维空间下 X轴/Roll 旋转矩阵
 *
 * @param theta Roll旋转角度（弧度制）
 * @return Struct_Matrix 3x3的旋转矩阵
 */
Struct_Matrix Calculate_3D_Roll_Matrix(float theta)
{
    Struct_Matrix matrix = Create_Matrix(3, 3);

    matrix.data[0 * matrix.columns + 0] = 1;
    matrix.data[0 * matrix.columns + 1] = 0;
    matrix.data[0 * matrix.columns + 2] = 0;

    matrix.data[1 * matrix.columns + 0] = 0;
    matrix.data[1 * matrix.columns + 1] = cosf(theta);
    matrix.data[1 * matrix.columns + 2] = -sinf(theta);

    matrix.data[2 * matrix.columns + 0] = 0;
    matrix.data[2 * matrix.columns + 1] = sinf(theta);
    matrix.data[2 * matrix.columns + 2] = cosf(theta);

    return matrix;
}

/**
 * @brief 外旋矩阵：按照 Z-Y-X 顺序绕世界坐标系旋转
 *
 * @param alpha 绕Z轴旋转的角度（弧度制）
 * @param beta  绕Y轴旋转的角度（弧度制）
 * @param theta 绕X轴旋转的角度（弧度制）
 * @return Struct_Matrix 3x3的旋转矩阵
 */
Struct_Matrix Calculate_Extrinsic_Rotation_Matrix(float alpha, float beta, float theta)
{
    Struct_Matrix matrix = Create_Matrix(3, 3);

    float c_a = cosf(alpha), s_a = sinf(alpha);
    float c_b = cosf(beta),  s_b = sinf(beta);
    float c_t = cosf(theta), s_t = sinf(theta);

    matrix.data[0 * matrix.columns + 0] = c_b * c_a;
    matrix.data[0 * matrix.columns + 1] = -c_b * s_a;
    matrix.data[0 * matrix.columns + 2] = s_b;

    matrix.data[1 * matrix.columns + 0] = s_t * s_b * c_a + c_t * s_a;
    matrix.data[1 * matrix.columns + 1] = -s_t * s_b * s_a + c_t * c_a;
    matrix.data[1 * matrix.columns + 2] = -s_t * c_b;

    matrix.data[2 * matrix.columns + 0] = -c_t * s_b * c_a + s_t * s_a;
    matrix.data[2 * matrix.columns + 1] = c_t * s_b * s_a + s_t * c_a;
    matrix.data[2 * matrix.columns + 2] = c_t * c_b;

    return matrix;
}

/**
 * @brief 内旋矩阵：按照 Yaw-Pitch-Roll 顺序绕物体坐标系旋转
 *
 * @param alpha Yaw旋转角度（弧度制）
 * @param beta  Pitch旋转角度（弧度制）
 * @param theta Roll旋转角度（弧度制）
 * @return Struct_Matrix 3x3的旋转矩阵
 */
Struct_Matrix Calculate_Intrinsic_Rotation_Matrix(float alpha, float beta, float theta)
{
    Struct_Matrix matrix = Create_Matrix(3, 3);

    float c_a = cosf(alpha), s_a = sinf(alpha);
    float c_b = cosf(beta),  s_b = sinf(beta);
    float c_t = cosf(theta), s_t = sinf(theta);

    matrix.data[0 * matrix.columns + 0] = c_a * c_b;
    matrix.data[0 * matrix.columns + 1] = -s_a * c_t + c_a * s_b * s_t;
    matrix.data[0 * matrix.columns + 2] = s_a * s_t + c_a * s_b * c_t;

    matrix.data[1 * matrix.columns + 0] = s_a * c_b;
    matrix.data[1 * matrix.columns + 1] = c_a * c_t + s_a * s_b * s_t;
    matrix.data[1 * matrix.columns + 2] = -c_a * s_t + s_a * s_b * c_t;

    matrix.data[2 * matrix.columns + 0] = -s_b;
    matrix.data[2 * matrix.columns + 1] = c_b * s_t;
    matrix.data[2 * matrix.columns + 2] = c_b * c_t;

    return matrix;
}

/**
 * @brief 根据IMU数据获取底盘旋转矩阵
 *
 * @param imu_data IMU数据结构体
 * @return Struct_Matrix 底盘旋转矩阵
 */
Struct_Matrix Get_Chassis_Rotation_Matrix(Struct_IMU_Data* imu_data)
{
    return Calculate_Intrinsic_Rotation_Matrix(imu_data->Yaw, imu_data->Pitch, imu_data->Roll);
}

/**
 * @brief 根据底盘IMU数据计算斜坡的倾角 (Pitch)
 *
 * @param imu_data 底盘IMU数据结构体
 * @return float 斜坡的倾角（弧度制）
 */
float Get_Slope_Pitch(Struct_IMU_Data* imu_data)
{
    // 1. 获取底盘的旋转矩阵 C
    Struct_Matrix C = Get_Chassis_Rotation_Matrix(imu_data);

    // 2. 提取矩阵C的第三列，即z轴方向向量 c_z (c_zx, c_zy, c_zz)
    float c_zx = C.data[0 * C.columns + 2];
    float c_zy = C.data[1 * C.columns + 2];
    float c_zz = C.data[2 * C.columns + 2];

    // 3. 根据公式计算斜坡Pitch角 beta_S
    // 使用 atan2f 能够更好地处理分母为0的情况并确保值域落在正确的象限
    float beta_s = atan2f(-sqrtf(c_zx * c_zx + c_zy * c_zy), c_zz);

    // 4. 释放矩阵内存
    Free_Matrix(&C);

    return beta_s;
}

/**
 * @brief 根据底盘IMU数据计算斜坡的朝向 (Yaw)
 *
 * @param imu_data 底盘IMU数据结构体
 * @return float 斜坡的朝向（弧度制）
 */
float Get_Slope_Yaw(Struct_IMU_Data* imu_data)
{
    // 1. 获取底盘的旋转矩阵 C
    Struct_Matrix C = Get_Chassis_Rotation_Matrix(imu_data);

    // 2. 提取矩阵C的第三列，即z轴方向向量 c_z
    float c_zx = C.data[0 * C.columns + 2];
    float c_zy = C.data[1 * C.columns + 2];
    float c_zz = C.data[2 * C.columns + 2];

    // 3. 先计算出斜坡的倾角 beta_S
    float beta_s = atan2f(-sqrtf(c_zx * c_zx + c_zy * c_zy), c_zz);

    float alpha_s = 0.0f;
    float sin_beta_s = sinf(beta_s);

    // 4. 根据公式反解斜坡的 Yaw 角 alpha_S
    // 逻辑：c_zy = sin(alpha_s) * sin(beta_s)
    //      c_zx = cos(alpha_s) * sin(beta_s)
    // 防止机器在平地上（倾角接近0）导致除以0，加入容差判断
    if (fabsf(sin_beta_s) > 1e-6f)
    {
        alpha_s = atan2f(c_zy / sin_beta_s, c_zx / sin_beta_s);
    }
    else
    {
        // 如果在平地上，斜坡朝向无意义，默认返回0
        alpha_s = 0.0f;
    }

    // 5. 释放矩阵内存
    Free_Matrix(&C);

    return alpha_s;
}

/**
 * @brief 计算底盘在斜坡上的相对朝向 (alpha_C - alpha_S)
 *
 * @param imu_data 底盘IMU数据结构体
 * @return float 底盘在斜坡上的相对朝向（弧度制）
 */
float Get_Chassis_Yaw_Relative_To_Slope(Struct_IMU_Data* imu_data)
{
    // 1. 获取底盘在世界坐标系下的 Yaw 角 (alpha_C)
    float alpha_c = imu_data->Yaw;

    // 2. 调用刚才写好的函数，获取斜坡在世界坐标系下的 Yaw 角 (alpha_S)
    float alpha_s = Get_Slope_Yaw(imu_data);

    // 3. 返回两者的差值 (c - s)
    return alpha_c - alpha_s;
}

/**
 * @brief 矩阵乘法 C = A * B
 *
 * @param A 左矩阵
 * @param B 右矩阵
 * @return Struct_Matrix 乘积矩阵
 */
Struct_Matrix Multiply_Matrix(Struct_Matrix A, Struct_Matrix B)
{
    // 确保矩阵维度可以相乘 (A的列数等于B的行数)
    if (A.columns != B.rows)
    {
        // 返回一个空矩阵作为错误处理
        return Create_Matrix(0, 0);
    }

    Struct_Matrix result = Create_Matrix(A.rows, B.columns);

    for (int i = 0; i < A.rows; i++)
    {
        for (int j = 0; j < B.columns; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < A.columns; k++)
            {
                sum += A.data[i * A.columns + k] * B.data[k * B.columns + j];
            }
            result.data[i * result.columns + j] = sum;
        }
    }

    return result;
}

/**
 * @brief 根据底盘IMU和云台电机编码器角度，计算云台的全局旋转矩阵
 *
 * @param imu_data     底盘IMU数据结构体指针
 * @param encoder_data 云台电机编码器数据结构体指针
 * @return Struct_Matrix 云台在世界坐标系下的旋转矩阵 G
 */
Struct_Matrix Get_Gimbal_Rotation_Matrix(Struct_IMU_Data* imu_data, Struct_Gimbal_Encoder_Data* encoder_data)
{
    // 1. 获取底盘相对于世界坐标系的旋转矩阵 C
    Struct_Matrix C = Get_Chassis_Rotation_Matrix(imu_data);

    // 2. 获取云台相对于底盘的旋转矩阵 ^C G
    // 云台转动属于内旋
    // 通常步兵机器人的云台只具有 Yaw 和 Pitch 两个自由度，因此 Roll 传入 0.0f。
    // 如果您的云台有 Roll 轴，请将 0.0f 替换为 encoder_data->Roll
    Struct_Matrix C_G = Calculate_Intrinsic_Rotation_Matrix(encoder_data->Yaw, encoder_data->Pitch, 0.0f);

    // 3. 计算云台相对于世界坐标系的旋转矩阵 G = C * ^C G
    Struct_Matrix G = Multiply_Matrix(C, C_G);

    // 4. 释放过程中产生的中间矩阵内存，防止内存泄漏
    Free_Matrix(&C);
    Free_Matrix(&C_G);

    // 5. 返回最终的云台旋转矩阵
    return G;
}

