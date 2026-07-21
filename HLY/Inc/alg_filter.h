//
// Created by 34285 on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_FILTER_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_FILTER_H

#include <stdint.h>
#include <math.h>
#include <stdlib.h>

// 采样频率
#define FOURIER_FILTER_DEFAULT_SAMPLING_FREQUENCY (1000.0f)
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 滤波器类型
 */
typedef enum
{
    Filter_Fourier_Type_LOWPASS = 0,
    Filter_Fourier_Type_HIGHPASS,
    Filter_Fourier_Type_BANDPASS,
    Filter_Fourier_Type_BANDSTOP,
} Enum_Filter_Fourier_Type;

/**
 * @brief Fourier滤波器结构体
 */
typedef struct
{
    // 输入限幅
    float Value_Constrain_Low;
    float Value_Constrain_High;

    // 滤波器类型
    Enum_Filter_Fourier_Type Filter_Fourier_Type;
    // 滤波器特征低频
    float Frequency_Low;
    // 滤波器特征高频
    float Frequency_High;
    // 滤波器采样频率
    float Sampling_Frequency;
    // 滤波器阶数
    uint32_t Filter_Fourier_Order;

    // 卷积系统函数向量 (动态分配)
    float *System_Function;
    // 输入信号向量 (动态分配)
    float *Input_Signal;

    // 新数据指示向量
    uint8_t Signal_Flag;
    // 输出值
    float Out;
} Struct_Filter_Fourier;

/**
 * @brief Kalman滤波器结构体
 */
typedef struct
{
    // 测量误差
    float Error_Measure;
    // 估计误差
    float Error_Estimate;
    // 增益
    float Kalman_Gain;
    // 输出值
    float Out;
    // 当前值
    float Now;
} Struct_Filter_Kalman;

/* Fourier 滤波器函数声明 */
void Init_Filter_Fourier(Struct_Filter_Fourier *filter, uint32_t order, float constrain_low, float constrain_high, Enum_Filter_Fourier_Type type, float freq_low, float freq_high, float sampling_freq);
void Free_Filter_Fourier(Struct_Filter_Fourier *filter);
float Get_Filter_Fourier_Out(Struct_Filter_Fourier *filter);
void Set_Filter_Fourier_Now(Struct_Filter_Fourier *filter, float now);
void Filter_Fourier_TIM_Calculate_PeriodElapsedCallback(Struct_Filter_Fourier *filter);

/* Kalman 滤波器函数声明 */
void Init_Filter_Kalman(Struct_Filter_Kalman *filter, float error_measure, float now, float error_estimate);
float Get_Filter_Kalman_Out(Struct_Filter_Kalman *filter);
void Set_Filter_Kalman_Now(Struct_Filter_Kalman *filter, float now);
void Filter_Kalman_TIM_Calculate_PeriodElapsedCallback(Struct_Filter_Kalman *filter);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_FILTER_H
