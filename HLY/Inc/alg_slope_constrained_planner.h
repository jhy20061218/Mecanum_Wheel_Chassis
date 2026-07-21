//
// Created by 34285 on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_SLOPE_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_SLOPE_H

#include <math.h>

/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 */
typedef enum
{
    Slope_First_REAL = 0,
    Slope_First_TARGET,
}Enum_Slope_First;

/**
 * @brief 斜坡函数数据结构体
 */
typedef struct
{
    // 输出值
    float Out;

    // 规划优先类型
    Enum_Slope_First Slope_First;

    // 当前规划值
    float Now_Planning;

    // 当前真实值
    float Now_Real;

    // 绝对值增量, 一次计算周期改变值
    float Increase_Value;

    // 绝对值减量, 一次计算周期改变值
    float Decrease_Value;

    // 目标值
    float Target;
}Struct_Slope;

void Init_Slope(Struct_Slope* slope, float increase_value, float decrease_value, Enum_Slope_First slope_first);
float Get_Slope_Out(Struct_Slope* slope);
void Set_Slope_Now_Real(Struct_Slope* slope, float now_real);
void Set_Slope_Increase_Value(Struct_Slope* slope, float increase_value);
void Set_Slope_Decrease_Value(Struct_Slope* slope, float decrease_value);
void Set_Slope_Target(Struct_Slope* slope, float target);
void Slope_TIM_Calculate_PeriodElapsedCallback(Struct_Slope* slope);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_SLOPE_H
