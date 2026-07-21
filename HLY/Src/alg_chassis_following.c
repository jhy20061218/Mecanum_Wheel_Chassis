//
// Created by 34285 on 2026/7/20.
//
#include "alg_chassis_following.h"

/**
 * @brief 初始化底盘跟随控制器参数
 *
 * @param follow     底盘跟随结构体指针
 * @param kp         比例系数
 * @param out_max    输出限幅
 * @param d_t        PID计算周期 (秒)
 * @param dead_zone  死区范围 (推荐 PI/12)
 */
void Init_Chassis_Follow(Struct_Chassis_Follow* follow, float kp, float out_max, float d_t, float dead_zone)
{
    if (follow == NULL)
    {
        return;
    }

    // 初始化内部变量
    follow->Out = 0.0f;
    follow->Normalized_Angle = 0.0f;

    // 调用提供的 PID_Init 接口初始化底层控制器
    // 仅配置 Kp, out_max, d_t, dead_zone，其余 I/D/F 和高级功能参数赋 0
    PID_Init(&follow->PID, kp, 0.0f, 0.0f, 0.0f, 0.0f, out_max, d_t,
             dead_zone, 0.0f, 0.0f, 0.0f, PID_D_First_Disable);

    // 底盘跟随的目标值始终为0
    PID_Set_Target(&follow->PID, 0.0f);
}

/**
 * @brief 获取输出值 (需要叠加到目标旋转速度上)
 *
 * @param follow 底盘跟随结构体指针
 * @return float 输出补偿值
 */
float Get_Chassis_Follow_Out(Struct_Chassis_Follow* follow)
{
    if (follow == NULL)
    {
        return 0.0f;
    }
    return follow->Out;
}

/**
 * @brief 底盘跟随周期计算函数
 *
 * @param follow        底盘跟随结构体指针
 * @param current_angle 当前值：底盘相对云台的角度
 */
void Chassis_Follow_Calculate(Struct_Chassis_Follow* follow, float current_angle)
{
    if (follow == NULL)
    {
        return;
    }

    // 1. 角度归一化到 (-PI, PI) 范围内
    float m = fmodf(current_angle, 2.0f * PI);

    if(m < -PI)
    {
        m = m + 2.0f * PI;
    }
    else if(m > PI)
    {
        m = m - 2.0f * PI;
    }

    follow->Normalized_Angle = m;

    // 2. 注入当前真实值
    PID_Set_Now(&follow->PID, m);

    // 3. 执行 PID 周期计算
    PID_TIM_Adjust_PeriodElapsedCallback(&follow->PID);

    // 4. 获取计算出的控制输出
    follow->Out = PID_Get_Out(&follow->PID);
}
