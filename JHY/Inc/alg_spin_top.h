//
// Created by JHY on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_SPIN_TOP_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_SPIN_TOP_H

#include "alg_mecanum_wheel_chassis.h"

// 小陀螺控制参数结构体
typedef struct
{
    float spin_omega;      // 小陀螺目标自转角速度 (单位: rad/s，决定陀螺转速)
    float feedforward_k;   // 延迟补偿前馈系数 k (单位: s)
} Struct_Spin_Top_Config;

void Calculate_Spin_Top_Target_Velocity(const Struct_Spin_Top_Config* config,
                                        const Struct_Target_Velocity* gimbal_target_vel,
                                        float gimbal_yaw_angle,
                                        float current_chassis_omega,
                                        Struct_Target_Velocity* chassis_target_vel);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_SPIN_TOP_H
