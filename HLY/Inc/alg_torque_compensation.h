//
// Created by 34285 on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_TORQUE_COMPENSATION_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_TORQUE_COMPENSATION_H

#include "alg_mecanum_wheel_chassis.h"

//重力加速度常量
#define GRAVITY_CONSTANT 9.81f

//阻力扭矩补偿参数结构体 - 低配版
typedef struct
{
    float t;                    // 标定的恒定阻力扭矩补偿值
    float continuous_threshold; // 一阶连续化的角速度阈值 (如 1.0 rad/s)
} Struct_Resistance_Config_Low;

//阻力扭矩补偿参数结构体 - 高配版
typedef struct
{
    float a0;                   // 二次多项式常数项
    float a1;                   // 二次多项式一次项系数
    float a2;                   // 二次多项式二次项系数
    float continuous_threshold; // 一阶连续化的角速度阈值 (如 1.0 rad/s)
    float limit_threshold;      // 前馈限幅的角速度阈值 (如 30.0 rad/s)
} Struct_Resistance_Config_High;

void Calculate_Uphill_Feedforward_Torque(const Struct_Chassis_Config* config,
                                         Struct_IMU_Data* imu_data,
                                         float total_mass,
                                         Struct_Wheel_Torque* feedforward_torque);
void Calculate_Uphill_Feedforward_Torque_High_End(const Struct_Chassis_Config* config,
                                                  Struct_IMU_Data* imu_data,
                                                  float total_mass,
                                                  float cog_height,
                                                  float max_f,
                                                  Struct_Wheel_Torque* feedforward_torque);

void Calculate_Resistance_Compensation_Low_End(const Struct_Resistance_Config_Low* config,
                                               const Struct_Wheel_Velocity* target_omega,
                                               Struct_Wheel_Torque* comp_torque);
void Calculate_Resistance_Compensation_High_End(const Struct_Resistance_Config_High* config,
                                                const Struct_Wheel_Velocity* target_omega,
                                                Struct_Wheel_Torque* comp_torque);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_TORQUE_COMPENSATION_H
