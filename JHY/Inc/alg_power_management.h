//
// Created by JHY on 2026/7/14.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_POWER_MANAGEMENT_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_POWER_MANAGEMENT_H

#include "alg_PID.h"

// 单个电机的运行状态（用于估计功率）
typedef struct
{
    float omega;   // 电机当前角速度 (单位: rad/s)
    float current; // 电机当前转矩电流 (单位: A，或归一化的控制量)
}Struct_Motor_State;

// 底盘四个电机的整体状态
typedef struct
{
    Struct_Motor_State LF; // 左前轮
    Struct_Motor_State LB; // 左后轮
    Struct_Motor_State RB; // 右后轮
    Struct_Motor_State RF; // 右前轮
}Struct_Chassis_Motors_State;

// 底盘电机功率估计模型的物理参数
// 模型参考：P_est = k1 * |omega * current| + k2 * current^2 + p0
typedef struct
{
    float k1; // 机械功率转化系数
    float k2; // 热损耗系数 (对应铜损 I^2 * R)
    float p0; // 静态功率损耗常数
}Struct_Power_Estimate_Config;

// 功率控制算法综合句柄
typedef struct
{
    Struct_PID power_pid;                         // 用于闭环控制功率余量的PID
    Struct_Power_Estimate_Config estimate_config; // 功率估计模型参数
}Struct_Power_Control;

// 基于二元函数二阶 Maclaurin 展开的功率估计模型参数
// 模型参考: P = k0 + k1 * I + k2 * ω + k3 * I * ω + k4 * I^2 + k5 * ω^2
typedef struct
{
    float k0;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
}Struct_Maclaurin_Estimate_Config;

// Maclaurin 功率控制算法综合句柄
typedef struct
{
    Struct_PID power_pid;                             // 用于闭环控制功率余量的PID
    Struct_Maclaurin_Estimate_Config estimate_config; // 功率估计模型参数 (6个系数)
}Struct_Maclaurin_Power_Control;

// 定义分离式的单电机状态结构体
typedef struct
{
    float dynamic_current; // 动力学解算部分对应的扭矩电流值
    float others_current; // 其他辅助算法部分对应的扭矩电流值 (如上坡补偿、阻力补偿)
    float omega; // 电机当前本体角速度
}Struct_Split_Motor_State;

// 定义分离式的底盘四轮状态结构体
typedef struct
{
    Struct_Split_Motor_State LF;
    Struct_Split_Motor_State LB;
    Struct_Split_Motor_State RB;
    Struct_Split_Motor_State RF;
}Struct_Chassis_Split_Motors_State;

//电机功率模型
void Power_Control_Init(Struct_Power_Control* power_ctrl,
                        const Struct_Power_Estimate_Config* est_config,
                        float pid_kp, float pid_ki, float pid_kd, float pid_dt);
float Estimate_Single_Motor_Power(const Struct_Power_Estimate_Config* config,
                                  const Struct_Motor_State* state);
float Calculate_Power_Attenuation_Factor(Struct_Power_Control* power_ctrl,
                                         float max_power,
                                         float current_power,
                                         float referee_limit_power,
                                         const Struct_Chassis_Motors_State* motors_state);

//功率控制
void Maclaurin_Power_Control_Init(Struct_Maclaurin_Power_Control* power_ctrl,
                                  const Struct_Maclaurin_Estimate_Config* est_config,
                                  float pid_kp, float pid_ki, float pid_kd, float pid_dt);
float Estimate_Single_Motor_Power_Maclaurin(const Struct_Maclaurin_Estimate_Config* config,
                                            const Struct_Motor_State* state);
float Calculate_Maclaurin_Power_Attenuation(Struct_Maclaurin_Power_Control* power_ctrl,
                                            float max_power,
                                            float current_power,
                                            float referee_limit_power,
                                            const Struct_Chassis_Motors_State* motors_state);

float Calculate_Maclaurin_Current_Attenuation(Struct_Maclaurin_Power_Control* power_ctrl,
                                              float max_power,
                                              float current_power,
                                              float referee_limit_power,
                                              const Struct_Chassis_Motors_State* motors_state);
float Calculate_Maclaurin_Special_Current_Attenuation(Struct_Maclaurin_Power_Control* power_ctrl,
                                                      float max_power,
                                                      float current_power,
                                                      float referee_limit_power,
                                                      const Struct_Chassis_Split_Motors_State* split_motors_state);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_POWER_MANAGEMENT_H
