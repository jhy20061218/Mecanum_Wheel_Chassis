//
// Created by JHY on 2026/6/19.
//

#ifndef MECANUM_WHEEL_CHASSIS_H
#define MECANUM_WHEEL_CHASSIS_H

#include <math.h>

typedef struct
{
    float a;         //左右轮距的一半 (单位: m)
    float b;         //前后轴距的一半 (单位: m)
    float R;         //车轮半径 (单位: m)
    float max_omega; //电机最大允许角速度 (单位: rad/s)，用于防饱和限幅
    float max_torque;//电机最大允许扭矩 (单位: N·m)，用于动态力矩防饱和限幅
}Struct_Chassis_Config;

// 底盘目标控制量
typedef struct
{
    float v_vector;  //底盘速度矢量
    float vx;        //横向平动速度 (单位: m/s，向右为正)
    float vy;        //纵向平动速度 (单位: m/s，向前为正)
    float omega;     //自转角速度 (单位: rad/s，逆时针为正)
}Struct_Target_Velocity;

//底盘目标受力与力矩 (通常由 PID 或 MPC 控制器计算得出)
typedef struct
{
    float Fx;        //横向期望受力 (单位: N，向右为正)
    float Fy;        //纵向期望受力 (单位: N，向前为正)
    float Mz;        //期望自转力矩 (单位: N·m，逆时针为正)
}Struct_Target_Force;

// 车轮输出角速度
typedef struct
{
    float LF;        //左前轮
    float LB;        //左后轮
    float RB;        //右后轮
    float RF;        //右前轮
}Struct_Wheel_Velocity;

//车轮输出目标扭矩
typedef struct
{
    float LF;        //左前轮电机扭矩 (单位: N·m)
    float LB;        //左后轮电机扭矩 (单位: N·m)
    float RB;        //右后轮电机扭矩 (单位: N·m)
    float RF;        //右前轮电机扭矩 (单位: N·m)
} Struct_Wheel_Torque;

void Mecanum_Wheel_Inverse_Kinematics(const Struct_Chassis_Config* config, const Struct_Target_Velocity* target,
                                      Struct_Wheel_Velocity* output);
void Mecanum_Wheel_Inverse_Dynamics(const Struct_Chassis_Config* config, const Struct_Target_Force* target_force,
                                    Struct_Wheel_Torque* output_torque);

#endif //MECANUM_WHEEL_CHASSIS_H
