//
// Created by 34285 on 2026/6/19.
//

#include "Mecanum_Wheel_Chassis.h"

/**
 * @brief 麦克纳姆轮逆运动学解算
 * @param config 底盘物理参数
 * @param target 期望的底盘三轴速度
 * @param output 解算后的四轮电机角速度输出
 */
void Mecanum_Wheel_Inverse_Kinematics(const Struct_Chassis_Config* config, const Struct_Target_Velocity* target,
                                      Struct_Wheel_Velocity* output)
{
    //基础运动学方程：计算车轮处所需的线速度 (m/s)
    float v_LF = target->vy + target->vx - target->omega * (config->a + config->b);
    float v_LB = target->vy - target->vx - target->omega * (config->a + config->b);
    float v_RB = target->vy + target->vx + target->omega * (config->a + config->b);
    float v_RF = target->vy - target->vx + target->omega * (config->a + config->b);

    //线速度转角速度：v = ω * R -> ω = v / R (rad/s)
    output->LF = v_LF / config->R;
    output->LB = v_LB / config->R;
    output->RB = v_RB / config->R;
    output->RF = v_RF / config->R;

    //关键安全防护：等比例缩放防饱和
    //寻找当前四个轮子中的最大转速绝对值
    float max_calc_omega = fabs(output->LF);
    if (fabs(output->LB) > max_calc_omega)
    {
        max_calc_omega = fabs(output->LB);
    }
    if (fabs(output->RB) > max_calc_omega)
    {
        max_calc_omega = fabs(output->RB);
    }
    if (fabs(output->RF) > max_calc_omega)
    {
        max_calc_omega = fabs(output->RF);
    }

    //严重警告：如果任一车轮转速超过电机物理极限而不做等比例缩放，底盘的受力合成将直接崩溃，导致不可预测的偏航甚至碰撞
    if (max_calc_omega > config->max_omega)
    {
        float scale = - config->max_omega / max_calc_omega;
        output->LF *= scale;
        output->LB *= scale;
        output->RB *= scale;
        output->RF *= scale;
    }
}
