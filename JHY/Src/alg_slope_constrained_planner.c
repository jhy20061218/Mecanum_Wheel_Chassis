//
// Created by JHY on 2026/7/16.
//
#include "alg_slope_constrained_planner.h"

/**
 * @brief 初始化斜坡参数
 *
 * @param slope          斜坡结构体指针
 * @param increase_value 增长最大幅度
 * @param decrease_value 降低最大幅度
 * @param slope_first    规划优先类型
 */
void Init_Slope(Struct_Slope* slope, float increase_value, float decrease_value, Enum_Slope_First slope_first)
{
    if (slope == NULL)
    {
        return;
    }
    slope->Increase_Value = increase_value;
    slope->Decrease_Value = decrease_value;
    slope->Slope_First = slope_first;

    // 初始化内部变量
    slope->Out = 0.0f;
    slope->Now_Planning = 0.0f;
    slope->Now_Real = 0.0f;
    slope->Target = 0.0f;
}

/**
 * @brief 获取输出值
 *
 * @param slope 斜坡结构体指针
 * @return float 输出值
 */
float Get_Slope_Out(Struct_Slope* slope)
{
    if (slope == NULL)
    {
        return 0.0f;
    }
    return slope->Out;
}

/**
 * @brief 设定当前真实值
 *
 * @param slope    斜坡结构体指针
 * @param now_real 当前真实值
 */
void Set_Slope_Now_Real(Struct_Slope* slope, float now_real)
{
    if (slope == NULL)
    {
        return;
    }
    slope->Now_Real = now_real;
}

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param slope          斜坡结构体指针
 * @param increase_value 绝对值增量
 */
void Set_Slope_Increase_Value(Struct_Slope* slope, float increase_value)
{
    if (slope == NULL)
    {
        return;
    }
    slope->Increase_Value = increase_value;
}

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param slope          斜坡结构体指针
 * @param decrease_value 绝对值减量
 */
void Set_Slope_Decrease_Value(Struct_Slope* slope, float decrease_value)
{
    if (slope == NULL)
    {
        return;
    }
    slope->Decrease_Value = decrease_value;
}

/**
 * @brief 设定目标值
 *
 * @param slope  斜坡结构体指针
 * @param target 目标值
 */
void Set_Slope_Target(Struct_Slope* slope, float target)
{
    if (slope == NULL)
    {
        return;
    }
    slope->Target = target;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 * @param slope 斜坡结构体指针
 */
void Slope_TIM_Calculate_PeriodElapsedCallback(Struct_Slope* slope)
{
    if (slope == NULL)
    {
        return;
    }

    // 规划为当前真实值优先的额外逻辑
    if (slope->Slope_First == Slope_First_REAL)
    {
        if ((slope->Target >= slope->Now_Real && slope->Now_Real >= slope->Now_Planning) ||
            (slope->Target <= slope->Now_Real && slope->Now_Real <= slope->Now_Planning))
        {
            slope->Out = slope->Now_Real;
        }
    }
    if (slope->Now_Planning > 0.0f)
    {
        if (slope->Target > slope->Now_Planning)
        {
            // 正向加速
            if (fabsf(slope->Now_Planning - slope->Target) > slope->Increase_Value)
            {
                slope->Out += slope->Increase_Value;
            }
            else
            {
                slope->Out = slope->Target;
            }
        }
        else if (slope->Target < slope->Now_Planning)
        {
            // 正向减速
            if (fabsf(slope->Now_Planning - slope->Target) > slope->Decrease_Value)
            {
                slope->Out -= slope->Decrease_Value;
            }
            else
            {
                slope->Out = slope->Target;
            }
        }
    }
    else if (slope->Now_Planning < 0.0f)
    {
        if (slope->Target < slope->Now_Planning)
        {
            // 反向加速
            if (fabsf(slope->Now_Planning - slope->Target) > slope->Increase_Value)
            {
                slope->Out -= slope->Increase_Value;
            }
            else
            {
                slope->Out = slope->Target;
            }
        }
        else if (slope->Target > slope->Now_Planning)
        {
            // 反向减速
            if (fabsf(slope->Now_Planning - slope->Target) > slope->Decrease_Value)
            {
                slope->Out += slope->Decrease_Value;
            }
            else
            {
                slope->Out = slope->Target;
            }
        }
    }
    else
    {
        if (slope->Target > slope->Now_Planning)
        {
            // 0值正加速
            if (fabsf(slope->Now_Planning - slope->Target) > slope->Increase_Value)
            {
                slope->Out += slope->Increase_Value;
            }
            else
            {
                slope->Out = slope->Target;
            }
        }
        else if (slope->Target < slope->Now_Planning)
        {
            // 0值负加速
            if (fabsf(slope->Now_Planning - slope->Target) > slope->Increase_Value)
            {
                slope->Out -= slope->Increase_Value;
            }
            else
            {
                slope->Out = slope->Target;
            }
        }
    }

    slope->Now_Planning = slope->Out;
}

