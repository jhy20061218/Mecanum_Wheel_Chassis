//
// Created by JHY on 2026/7/16.
//
#include "alg_spin_top.h"

/**
 * @brief 小陀螺模式下的底盘目标速度解算（带相位超前前馈补偿）
 *
 * @param config 小陀螺配置参数结构体指针 (包含自转角速度和前馈参数k)
 * @param gimbal_target_vel 操作手在云台坐标系下的期望目标平动速度 (通常由 WSAD 映射而来)
 * @param gimbal_yaw_angle 云台Yaw轴相对于底盘的转角 (单位: 弧度，θ)
 * @param current_chassis_omega 底盘当前的实际角速度 (单位: rad/s，来自陀螺仪 Z 轴数据)
 * @param chassis_target_vel 解算后输出的底盘坐标系下的目标速度，可直接送入底盘逆运动学函数
 */
void Calculate_Spin_Top_Target_Velocity(const Struct_Spin_Top_Config* config,
                                        const Struct_Target_Velocity* gimbal_target_vel,
                                        float gimbal_yaw_angle,
                                        float current_chassis_omega,
                                        Struct_Target_Velocity* chassis_target_vel)
{
    // 1. 计算前馈补偿后的等效角度
    // 原理: 底盘转向电机和云台响应存在延迟，直接使用当前角度会导致底盘小陀螺行进方向偏移
    // 引入前馈量: comp_angle = θ + k * ω
    float comp_angle = gimbal_yaw_angle + config->feedforward_k * current_chassis_omega;

    // 2. 提前计算二维旋转矩阵所需的三角函数值
    float cos_theta = cosf(comp_angle);
    float sin_theta = sinf(comp_angle);

    // 3. 速度矢量的旋转变换 (云台坐标系 -> 底盘坐标系)
    // Z轴旋转矩阵变换公式:
    // [ v_cx ] = [ cos(θ)  -sin(θ) ] * [ v_gx ]
    // [ v_cy ] = [ sin(θ)   cos(θ) ] * [ v_gy ]
    chassis_target_vel->vx = cos_theta * gimbal_target_vel->vx - sin_theta * gimbal_target_vel->vy;
    chassis_target_vel->vy = sin_theta * gimbal_target_vel->vx + cos_theta * gimbal_target_vel->vy;

    // 4. 叠加小陀螺的自转角速度
    // 小陀螺模式下，底盘要保持自身的快速旋转，所以角速度直接赋为设定的目标值
    chassis_target_vel->omega = config->spin_omega;

    // 5. (可选) 更新目标速度矢量的模长，为后续可能的安全限幅提供数据
    chassis_target_vel->v_vector = sqrtf(chassis_target_vel->vx * chassis_target_vel->vx +
                                         chassis_target_vel->vy * chassis_target_vel->vy);
}
