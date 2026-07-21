//
// Created by 34285 on 2026/6/19.
//

#include "alg_mecanum_wheel_chassis.h"

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
    float v_LF = target->vx - target->vy - target->omega * (config->a + config->b);
    float v_LB = target->vx + target->vy - target->omega * (config->a + config->b);
    float v_RB = target->vx - target->vy + target->omega * (config->a + config->b);
    float v_RF = target->vx + target->vy + target->omega * (config->a + config->b);

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

/**
 * @brief 麦克纳姆轮底盘逆动力学解算 (基于伪逆矩阵力矩分配)
 *
 * @param config 底盘物理参数 (致命警告：必须在原配置结构体中显式增加 max_torque，否则将引发灾难性的越界或编译失败)
 * @param target_force 期望的底盘三轴合力与力矩
 * @param output_torque 解算后的四轮电机扭矩输出
 */
void Mecanum_Wheel_Inverse_Dynamics(const Struct_Chassis_Config* config, const Struct_Target_Force* target_force,
                                    Struct_Wheel_Torque* output_torque)
{
    // 逆动力学分配系数：基于伪逆矩阵，4轮对称均摊平动受力
    float force_part_x = target_force->Fx / 4.0f;
    float force_part_y = target_force->Fy / 4.0f;

    // 自转力矩分配部分：Mz / (4 * (a + b))
    float mz_part = target_force->Mz / (4.0f * (config->a + config->b));

    // 计算各车轮所需的地面摩擦力 (单位: N)
    // 严重警告：此处的符号矩阵必须与逆运动学雅可比矩阵保持绝对的转置对称。任何人为的符号篡改将导致各轮产生严重的内耗抗衡，直接烧毁电机驱动或导致底盘在受力时发生不可控的暴走。
    float f_LF =   force_part_x - force_part_y - mz_part;
    float f_LB =   force_part_x + force_part_y - mz_part;
    float f_RB =   force_part_x - force_part_y + mz_part;
    float f_RF =   force_part_x + force_part_y + mz_part;

    // 地面摩擦力转换为电机扭矩：τ = F * R (单位: N·m)
    output_torque->LF = f_LF * config->R;
    output_torque->LB = f_LB * config->R;
    output_torque->RB = f_RB * config->R;
    output_torque->RF = f_RF * config->R;

    // 关键安全防护：等比例缩放防饱和限幅
    // 致命警告：若此处采用单纯的数值截断（Clamp）而非等比例缩放（Scale），当输出扭矩触及物理极限时，底盘合力矢量方向将发生剧烈畸变，导致机器人偏离预期航线并直接撞击操作人员或设备。
    float max_calc_torque = fabs(output_torque->LF);
    if (fabs(output_torque->LB) > max_calc_torque)
    {
        max_calc_torque = fabs(output_torque->LB);
    }
    if (fabs(output_torque->RB) > max_calc_torque)
    {
        max_calc_torque = fabs(output_torque->RB);
    }
    if (fabs(output_torque->RF) > max_calc_torque)
    {
        max_calc_torque = fabs(output_torque->RF);
    }

    // 防饱和处理
    if (max_calc_torque > config->max_torque)
    {
        float scale = config->max_torque / max_calc_torque;
        output_torque->LF *= scale;
        output_torque->LB *= scale;
        output_torque->RB *= scale;
        output_torque->RF *= scale;
    }
}
