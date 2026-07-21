//
// Created by 34285 on 2026/7/16.
//
#include "alg_filter.h"

/* 私有数学辅助函数 */
/**
 * @brief 归一化 sinc 函数 sin(x)/x 的本地实现
 *
 * 用于 Fourier 滤波器系统函数的频域采样计算。
 * 当 x 接近 0 时返回极限值 1.0，避免除零错误。
 *
 * @param x 输入值（通常为归一化频率与滤波器半阶数的乘积）
 * @return static float sinc(x) 的值，范围约在 [-0.217, 1.0] 之间
 */
static float Math_Sinc_C(float x)
{
    if (fabsf(x) < 1e-6f) return 1.0f;
    return sinf(x) / x;
}

/**
 * @brief 将浮点数值约束在指定范围内（原地修改）
 *
 * 若 *val 小于 min_val 则置为 min_val，若大于 max_val 则置为 max_val，
 * 若在范围内则不做修改。用于滤波器输入信号的限幅保护。
 *
 * @param val     待约束值的指针
 * @param min_val 下限值
 * @param max_val 上限值
 */
static void Math_Constrain_C(float *val, float min_val, float max_val)
{
    if (*val < min_val) *val = min_val;
    else if (*val > max_val) *val = max_val;
}

/**
 * @brief 初始化Fourier滤波器
 */
void Init_Filter_Fourier(Struct_Filter_Fourier *filter, uint32_t order, float constrain_low, float constrain_high, Enum_Filter_Fourier_Type type, float freq_low, float freq_high, float sampling_freq)
{
    if (filter == NULL) return;

    filter->Filter_Fourier_Order = order;
    filter->Value_Constrain_Low = constrain_low;
    filter->Value_Constrain_High = constrain_high;
    filter->Filter_Fourier_Type = type;
    filter->Frequency_Low = freq_low;
    filter->Frequency_High = freq_high;
    filter->Sampling_Frequency = sampling_freq;
    filter->Signal_Flag = 0;
    filter->Out = 0.0f;

    // 动态分配内存
    filter->System_Function = (float *)calloc(order + 1, sizeof(float));
    filter->Input_Signal = (float *)calloc(order + 1, sizeof(float));

    float system_function_sum = 0.0f;
    float omega_low = 2.0f * PI * filter->Frequency_Low / filter->Sampling_Frequency;
    float omega_high = 2.0f * PI * filter->Frequency_High / filter->Sampling_Frequency;

    // 计算滤波器系统
    for (uint32_t i = 0; i < filter->Filter_Fourier_Order + 1; i++)
    {
        float x = i - filter->Filter_Fourier_Order / 2.0f;
        switch (filter->Filter_Fourier_Type)
        {
            case Filter_Fourier_Type_LOWPASS:
                filter->System_Function[i] = omega_low / PI * Math_Sinc_C(x * omega_low);
                break;
            case Filter_Fourier_Type_HIGHPASS:
                filter->System_Function[i] = Math_Sinc_C(x * PI) - omega_high / PI * Math_Sinc_C(x * omega_high);
                break;
            case Filter_Fourier_Type_BANDPASS:
                filter->System_Function[i] = omega_high / PI * Math_Sinc_C(x * omega_high) - omega_low / PI *
                                             Math_Sinc_C(x * omega_low);
                break;
            case Filter_Fourier_Type_BANDSTOP:
                filter->System_Function[i] = Math_Sinc_C(x * PI) + omega_low / PI * Math_Sinc_C(x * omega_low) -
                                             omega_high / PI * Math_Sinc_C(x * omega_high);
                break;
        }
        system_function_sum += filter->System_Function[i];
    }

    // 归一化
    if (fabsf(system_function_sum) > 1e-6f)
    {
        for (uint32_t i = 0; i < filter->Filter_Fourier_Order + 1; i++)
        {
            filter->System_Function[i] /= system_function_sum;
        }
    }
}

/**
 * @brief 释放Fourier滤波器内存
 */
void Free_Filter_Fourier(Struct_Filter_Fourier *filter)
{
    if (filter != NULL)
    {
        if (filter->System_Function != NULL) free(filter->System_Function);
        if (filter->Input_Signal != NULL) free(filter->Input_Signal);
        filter->System_Function = NULL;
        filter->Input_Signal = NULL;
    }
}

/**
 * @brief 获取 Fourier 滤波器当前的卷积输出值
 *
 * 返回最近一次 Filter_Fourier_TIM_Calculate_PeriodElapsedCallback 计算得到的滤波结果。
 *
 * @param filter Fourier 滤波器结构体指针
 * @return float 滤波后的输出值，filter 为空时返回 0.0
 */
float Get_Filter_Fourier_Out(Struct_Filter_Fourier *filter)
{
    return (filter != NULL) ? filter->Out : 0.0f;
}

/**
 * @brief 向 Fourier 滤波器输入一个当前采样值
 *
 * 将当前测量值写入环形缓冲区 Input_Signal 的当前位置，并对信号进行限幅约束。
 * 写入后采样指针 Signal_Flag 递增，到达缓冲区末尾时自动回绕至起始位置。
 *
 * @param filter Fourier 滤波器结构体指针
 * @param now    当前输入的采样值
 */
void Set_Filter_Fourier_Now(Struct_Filter_Fourier *filter, float now)
{
    if (filter == NULL || filter->Input_Signal == NULL) return;
    if (filter->Value_Constrain_Low != 0.0f || filter->Value_Constrain_High != 0.0f)
    {
        Math_Constrain_C(&now, filter->Value_Constrain_Low, filter->Value_Constrain_High);
    }

    filter->Input_Signal[filter->Signal_Flag] = now;
    filter->Signal_Flag++;
    if (filter->Signal_Flag == filter->Filter_Fourier_Order + 1)
    {
        filter->Signal_Flag = 0;
    }
}

/**
 * @brief Fourier 滤波器定时卷积计算回调
 *
 * 在每次定时器周期到达时执行 FIR 卷积运算：
 * 将系统函数 System_Function（即滤波器系数 h[0..N]）与环形缓冲区内存储的
 * 输入信号 Input_Signal 进行逐点乘加，结果写入 filter->Out。
 * 卷积采用循环移位索引，保证因果系统在离散时间域的正确滤波。
 *
 * @param filter Fourier 滤波器结构体指针
 */
void Filter_Fourier_TIM_Calculate_PeriodElapsedCallback(Struct_Filter_Fourier *filter)
{
    if (filter == NULL || filter->System_Function == NULL || filter->Input_Signal == NULL) return;

    filter->Out = 0.0f;
    for (uint32_t i = 0; i < filter->Filter_Fourier_Order + 1; i++)
    {
        uint32_t index = (filter->Signal_Flag + i) % (filter->Filter_Fourier_Order + 1);
        filter->Out += filter->System_Function[i] * filter->Input_Signal[index];
    }
}

/**
 * @brief 初始化Kalman滤波器
 */
void Init_Filter_Kalman(Struct_Filter_Kalman *filter, float error_measure, float now, float error_estimate)
{
    if (filter == NULL) return;
    filter->Error_Measure = error_measure;
    filter->Now = now;
    filter->Error_Estimate = error_estimate;
    filter->Kalman_Gain = 0.0f;
    filter->Out = 0.0f;
}

/**
 * @brief 获取 Kalman 滤波器当前的最优估计输出值
 *
 * 返回最近一次 Filter_Kalman_TIM_Calculate_PeriodElapsedCallback 计算得到的
 * 状态估计结果。
 *
 * @param filter Kalman 滤波器结构体指针
 * @return float 最优估计值，filter 为空时返回 0.0
 */
float Get_Filter_Kalman_Out(Struct_Filter_Kalman *filter)
{
    return (filter != NULL) ? filter->Out : 0.0f;
}

/**
 * @brief 向 Kalman 滤波器输入当前测量值
 *
 * 将最新的传感器测量值写入滤波器内部状态 Now，
 * 该值将在下一次滤波计算回调中被用于更新最优估计。
 *
 * @param filter Kalman 滤波器结构体指针
 * @param now    当前测量值
 */
void Set_Filter_Kalman_Now(Struct_Filter_Kalman *filter, float now)
{
    if (filter != NULL) filter->Now = now;
}

/**
 * @brief Kalman 滤波器定时更新回调：执行一步预测-更新迭代
 *
 * 按一维 Kalman 滤波公式执行一次完整的递推更新：
 * 1. 计算 Kalman 增益 K = Error_Estimate / (Error_Estimate + Error_Measure)
 * 2. 更新最优估计 Out = Out + K * (Now - Out)
 * 3. 更新估计误差 Error_Estimate = (1 - K) * Error_Estimate
 *
 * 本函数应在定时器周期到达时被调用以持续追踪传感器信号。
 *
 * @param filter Kalman 滤波器结构体指针
 */
void Filter_Kalman_TIM_Calculate_PeriodElapsedCallback(Struct_Filter_Kalman *filter)
{
    if (filter == NULL) return;

    filter->Kalman_Gain = filter->Error_Estimate / (filter->Error_Estimate + filter->Error_Measure);
    filter->Out = filter->Out + filter->Kalman_Gain * (filter->Now - filter->Out);
    filter->Error_Estimate = (1.0f - filter->Kalman_Gain) * filter->Error_Estimate;
}


