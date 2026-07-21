//
// Created by 34285 on 2026/7/20.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_CHASSIS_TRACKING_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_CHASSIS_TRACKING_H

#include "alg_PID.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief 底盘跟随控制器数据结构体
 */
typedef struct
{
    // 控制器输出值 (底盘目标旋转速度的补偿值)
    float Out;

    // 当前底盘相对云台的角度归一化结果
    float Normalized_Angle;

    // 内嵌标准 PID 结构体实例
    Struct_PID PID;
}Struct_Chassis_Follow;

void Init_Chassis_Follow(Struct_Chassis_Follow* follow, float kp, float out_max, float d_t, float dead_zone);
float Get_Chassis_Follow_Out(Struct_Chassis_Follow* follow);
void Chassis_Follow_Calculate(Struct_Chassis_Follow* follow, float current_angle);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_CHASSIS_TRACKING_H
