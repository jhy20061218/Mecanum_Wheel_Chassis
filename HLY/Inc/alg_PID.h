//
// Created by 34285 on 2026/4/19.
//

#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "drv_math.h"

//微分先行
typedef enum
{
    PID_D_First_Disable = 0,
    PID_D_First_Enable = 1,
}Enum_PID_D_First;

//PID结构体
typedef struct
{
    //读变量
    //输出值
    float Out;

    //读写变量
    //积分值
    float I_Error;

    //目标值
    float Target;
    //当前值
    float Now;

    //写变量
    //比例系数
    float K_P;
    //积分系数
    float K_I;
    //微分系数
    float K_D;
    //前馈系数
    float K_F;

    //内部变量
    //之前的当前值
    float Pre_Now;
    //之前的目标值
    float Pre_Target;
    //之前的输出值
    float Pre_Out;
    //之前的误差
    float Pre_Error;

    //初始化相关常量
    //PID计时周期
    float D_T;
    //死区，Error在其绝对值内不输出
    float Dead_Zone;
    //微分先行
    Enum_PID_D_First D_First;

    //积分限幅，0为不限幅
    float I_Out_Max;
    //输出限幅，0为不限幅
    float Out_Max;

    //变速积分定速内段阈值，0为不限制
    float I_Variable_Speed_A;
    //变速积分变速区间，0为不限制
    float I_Variable_Speed_B;
    //积分分离阈值，需为正数，0为不限制
    float I_Separate_Threshold;
}Struct_PID;


void PID_Init(Struct_PID *pid, float k_p, float k_i, float k_d, float k_f, float i_out_max,
              float out_max, float d_t,float dead_zone, float i_variable_a, float i_variable_b,
              float i_separate_threshold, Enum_PID_D_First d_first);

float PID_Get_I_Error(Struct_PID *pid);
float PID_Get_Out(Struct_PID *pid);

void PID_Set_K_P(Struct_PID *pid, float k_p);
void PID_Set_K_I(Struct_PID *pid, float k_i);
void PID_Set_K_D(Struct_PID *pid, float k_d);
void PID_Set_K_F(Struct_PID *pid, float k_f);
void PID_Set_I_Out_Max(Struct_PID *pid, float i_out_max);
void PID_Set_Out_Max(Struct_PID *pid, float out_max);
void PID_Set_I_Variable_Speed_A(Struct_PID *pid, float i_variable_a);
void PID_Set_I_Variable_Speed_B(Struct_PID *pid, float i_variable_b);
void PID_Set_I_Separate_Threshold(Struct_PID *pid, float i_separate_threshold);
void PID_Set_Target(Struct_PID *pid, float target);
void PID_Set_Now(Struct_PID *pid, float now);
void PID_Set_I_Error(Struct_PID *pid, float error);

void PID_TIM_Adjust_PeriodElapsedCallback(Struct_PID *pid);

#endif //__ALG_PID_H__
