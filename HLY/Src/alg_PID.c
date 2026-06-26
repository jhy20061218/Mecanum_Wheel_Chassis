//
// Created by 34285 on 2026/4/19.
//

#include "alg_PID.h"

/**
 * @brief 初始化PID结构体并设置参数
 *
 * @param pid PID结构体指针
 * @param k_p 比例系数
 * @param k_i 积分系数
 * @param k_d 微分系数
 * @param k_f 前馈系数
 * @param i_out_max 积分限幅（0为不限）
 * @param out_max 输出限幅（0为不限）
 * @param d_t PID周期（秒）
 * @param dead_zone 死区，误差绝对值低于此值视为0
 * @param i_variable_a 变速积分阈值A（0为不启用）
 * @param i_variable_b 变速积分区间B（0为不启用）
 * @param i_separate_threshold 积分分离阈值（0为不启用）
 * @param d_first 是否微分先行（Enum_PID_D_First）
 */
void PID_Init(Struct_PID *pid, float k_p, float k_i, float k_d, float k_f, float i_out_max,
              float out_max, float d_t,float dead_zone, float i_variable_a, float i_variable_b,
              float i_separate_threshold, Enum_PID_D_First d_first)
{
    //初始化内部变量为0
    pid->Pre_Now = 0.0f;
    pid->Pre_Target = 0.0f;
    pid->Pre_Out = 0.0f;
    pid->Pre_Error = 0.0f;
    pid->Out = 0.0f;
    pid->I_Error = 0.0f;
    pid->Target = 0.0f;
    pid->Now = 0.0f;

    //赋值配置参数
    pid->K_P = k_p;
    pid->K_I = k_i;
    pid->K_D = k_d;
    pid->K_F = k_f;
    pid->I_Out_Max = i_out_max;
    pid->Out_Max = out_max;
    pid->D_T = d_t;
    pid->Dead_Zone = dead_zone;
    pid->I_Variable_Speed_A = i_variable_a;
    pid->I_Variable_Speed_B = i_variable_b;
    pid->I_Separate_Threshold = i_separate_threshold;
    pid->D_First = d_first;
}

/**
 * @brief 获取PID的积分值
 *
 * @param pid PID结构体指针
 * @return 积分值
 */
float PID_Get_I_Error(Struct_PID *pid)
{
    return pid->I_Error;
}

/**
 * @brief 获取PID的输出值
 *
 * @param pid PID结构体指针
 * @return 输出值
 */
float PID_Get_Out(Struct_PID *pid)
{
    return pid->Out;
}

/**
 * @brief 设置PID的比例系数
 *
 * @param pid PID结构体指针
 * @param k_p 比例系数
 */
void PID_Set_K_P(Struct_PID *pid, float k_p)
{
    pid->K_P = k_p;
}

/**
 * @brief 设置PID的积分系数
 *
 * @param pid PID结构体指针
 * @param k_i 积分系数
 */
void PID_Set_K_I(Struct_PID *pid, float k_i)
{
    pid->K_I = k_i;
}

/**
 * @brief 设置PID的微分系数
 *
 * @param pid PID结构体指针
 * @param k_d 微分系数
 */
void PID_Set_K_D(Struct_PID *pid, float k_d)
{
    pid->K_D = k_d;
}

/**
 * @brief 设置前馈系数
 *
 * @param pid PID结构体指针
 * @param k_f 前馈系数
 */
void PID_Set_K_F(Struct_PID *pid, float k_f)
{
    pid->K_F = k_f;
}

/**
 * @brief 设置积分限幅，0表示不限制
 *
 * @param pid PID结构体指针
 * @param i_out_max 积分限幅（0为不限）
 */
void PID_Set_I_Out_Max(Struct_PID *pid, float i_out_max)
{
    pid->I_Out_Max = i_out_max;
}

/**
 * @brief 设置输出限幅，0表示不限制
 *
 * @param pid PID结构体指针
 * @param out_max 输出限幅（0为不限）
 */
void PID_Set_Out_Max(Struct_PID *pid, float out_max)
{
    pid->Out_Max = out_max;
}

/**
 * @brief 设置变速积分的定速内段阈值（A），0表示不启用
 *
 * @param pid PID结构体指针
 * @param i_variable_a 阈值A（0为不启用）
 */
void PID_Set_I_Variable_Speed_A(Struct_PID *pid, float i_variable_a)
{
    pid->I_Variable_Speed_A = i_variable_a;
}

/**
 * @brief 设置变速积分的变速区间（B），0表示不启用
 *
 * @param pid PID结构体指针
 * @param i_variable_b 变速区间B（0为不启用）
 */
void PID_Set_I_Variable_Speed_B(Struct_PID *pid, float i_variable_b)
{
    pid->I_Variable_Speed_B = i_variable_b;
}

/**
 * @brief 设置积分分离阈值，误差大于该阈值时会清零积分（0为不启用）
 *
 * @param pid PID结构体指针
 * @param i_separate_threshold 积分分离阈值（>=0，0为不启用）
 */
void PID_Set_I_Separate_Threshold(Struct_PID *pid, float i_separate_threshold)
{
    pid->I_Separate_Threshold = i_separate_threshold;
}

/**
 * @brief 设置PID目标值（参考值）
 *
 * @param pid PID结构体指针
 * @param target 目标值
 */
void PID_Set_Target(Struct_PID *pid, float target)
{
    pid->Target = target;
}

/**
 * @brief 设置PID当前测量值
 *
 * @param pid PID结构体指针
 * @param now 当前值
 */
void PID_Set_Now(Struct_PID *pid, float now)
{
    pid->Now = now;
}

/**
 * @brief 手动设置积分误差（常用于清零积分）
 *
 * @param pid PID结构体指针
 * @param error 积分误差值
 */
void PID_Set_I_Error(Struct_PID *pid, float error)
{
    pid->I_Error = error;
}

/**
 * @brief PID计算周期回调函数，执行P/I/D/F计算并更新 pid->Out
 *
 * 本函数应在定时周期（pid->D_T）到达时被调用以更新控制量。
 *
 * @param pid PID结构体指针
 */
void PID_TIM_Adjust_PeriodElapsedCallback(Struct_PID *pid)
{
    //P输出
    float p_out = 0.0f;
    //I输出
    float i_out = 0.0f;
    //D输出
    float d_out = 0.0f;
    //F输出
    float f_out = 0.0f;
    //误差
    float error = 0.0f;
    //绝对值误差
    float abs_error = 0.0f;
    //线性变速积分
    float speed_ratio = 0.0f;

    error = pid->Target - pid->Now;
    abs_error = MATH_ABS(error);

    //判断死区
    if (abs_error < pid->Dead_Zone)
    {
        pid->Target = pid->Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    //计算P
    p_out = pid->K_P * error;

    //计算I
    if (pid->I_Variable_Speed_A == 0.0f && pid->I_Variable_Speed_B == 0.0f)
    {
        //非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        //变速积分
        if (abs_error < pid->I_Variable_Speed_A)
        {
            //误差小于A，正常积分
            speed_ratio = 1.0f;
        }
        else if (abs_error < pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)
        {
            //误差在A到A+B之间，线性递减
            speed_ratio = 1.0f -(abs_error - pid->I_Variable_Speed_A) / pid->I_Variable_Speed_B;
        }
        else
        {
            //误差大于A+B，停止积分
            speed_ratio = 0.0f;
        }
    }
    //积分限幅
    if (pid->I_Out_Max != 0.0f)
    {
        MATH_CONSTRAIN(&(pid->I_Error), -pid->I_Out_Max / pid->K_I, pid->I_Out_Max / pid->K_I);
    }

    if (pid->I_Separate_Threshold == 0.0f)
    {
        //没有积分分离
        pid->I_Error += speed_ratio * pid->D_T * error;
        i_out = pid->K_I * pid->I_Error;
    }
    else
    {
        //积分分离使能
        if (abs_error < pid->I_Separate_Threshold)
        {
            pid->I_Error += speed_ratio * pid->D_T * error;
            i_out = pid->K_I * pid->I_Error;
        }
        else
        {
            pid->I_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    //计算d项
    if (pid->D_First == PID_D_First_Disable)
    {
        //没有微分先行
        d_out = pid->K_D * (error - pid->Pre_Error) / pid->D_T;
    }
    else
    {
        //微分先行使能
        d_out = -pid->K_D * (pid->Now - pid->Pre_Now) / pid->D_T;
    }

    //计算前馈

    f_out = (pid->Target - pid->Pre_Target) * pid->K_F;

    //计算总共的输出

    pid->Out = p_out + i_out + d_out + f_out;

    //输出限幅
    if (pid->Out_Max != 0.0f)
    {
        MATH_CONSTRAIN(&(pid->Out), -pid->Out_Max, pid->Out_Max);
    }

    //为下一次的循环准备
    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = pid->Out;
    pid->Pre_Error = error;
}

