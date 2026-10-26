//
// Created by JHY on 2026/7/16.
//
#include "alg_torque_compensation.h"

/**
 * @brief 上坡扭矩补偿——低配版前馈解算
 *
 * @param config 底盘物理参数结构体指针
 * @param imu_data 底盘IMU数据结构体指针
 * @param total_mass 整车总质量 (单位: kg)
 * @param feedforward_torque 解算后输出的四轮电机扭矩前馈量
 */
void Calculate_Uphill_Feedforward_Torque(const Struct_Chassis_Config* config,
                                         Struct_IMU_Data* imu_data,
                                         float total_mass,
                                         Struct_Wheel_Torque* feedforward_torque)
{
    // 1. 获取底盘的倾角 βs 以及底盘相对于斜坡的转角 α
    float beta_s = Get_Slope_Pitch(imu_data);
    float alpha  = Get_Chassis_Yaw_Relative_To_Slope(imu_data);

    // 2. 计算克服重力分量的牵引力前馈向量 f 的大小
    // 公式: |f| = mg * sin(βs)
    float f_magnitude = total_mass * GRAVITY_CONSTANT * sinf(beta_s);

    // 3. 将牵引力向量 f 分解到底盘坐标系下
    // f 相对于底盘坐标系的方向为 -α
    // 根据底盘坐标系右手系定义（Fx向右为正，Fy向前为正，逆时针Yaw为正）：
    // 斜坡上坡方向 (0, 1) 从斜坡坐标系转换到底盘坐标系，需要乘以旋转矩阵 R(-α)
    // 推导得出:
    // Fx = f_mag * sin(α)
    // Fy = f_mag * cos(α)
    Struct_Target_Force feedforward_force;
    feedforward_force.Fx = f_magnitude * sinf(alpha);
    feedforward_force.Fy = f_magnitude * cosf(alpha);

    // 把 f 当作底盘目标牵引力，而底盘目标转矩为0
    feedforward_force.Mz = 0.0f;

    // 4. 通过动力学逆解算，得出上坡扭矩前馈
    Omni_Wheel_Inverse_Dynamics(config, &feedforward_force, feedforward_torque);
}

/**
 * @brief 上坡扭矩补偿——高配版前馈解算
 *
 * @param config 底盘物理参数结构体指针
 * @param imu_data 底盘IMU数据结构体指针
 * @param total_mass 整车总质量 (单位: kg)
 * @param cog_height 重心高度 (单位: m)
 * @param max_f 限制的最大补偿力 (单位: N)，用于防止某些特殊位姿时疯车
 * @param feedforward_torque 解算后输出的四轮电机扭矩前馈量
 */
void Calculate_Uphill_Feedforward_Torque_High_End(const Struct_Chassis_Config* config,
                                                  Struct_IMU_Data* imu_data,
                                                  float total_mass,
                                                  float cog_height,
                                                  float max_f,
                                                  Struct_Wheel_Torque* feedforward_torque)
{
    // 1. 获取底盘的旋转矩阵 C
    Struct_Matrix C = Get_Chassis_Rotation_Matrix(imu_data);

    // 2. 提取矩阵C的第三行，这代表世界坐标系Z轴（重力反方向）在底盘坐标系下的投影
    // C20, C21, C22 决定了重力向量在底盘三轴上的分量
    float c20 = C.data[2 * C.columns + 0];
    float c21 = C.data[2 * C.columns + 1];
    float c22 = C.data[2 * C.columns + 2];

    // 释放矩阵内存，防止内存泄漏
    Free_Matrix(&C);

    // 3. 计算重力投影点 P (px, py)
    // 重心高度 h 乘上 x/y 分量。严格几何推导需除以 c22 以投影到底盘平面。
    float px = 0.0f;
    float py = 0.0f;

    // 避免底盘翻车(c22 <= 0)或极度倾斜时的异常除0错误
    if (c22 > 0.05f)
    {
        px = -cog_height * (c20 / c22);
        py = -cog_height * (c21 / c22);
    }

    // 4. 最好限制重力投影点的范围：不要离四个轮组投影点太近
    // 轮子在 X/Y 轴上的投影距离 W_L
    float W_L = config->r_center * SQRT_2_OVER_2;
    float limit = 0.8f * W_L; // 限制在四个轮子连线矩形的 80% 范围内

    if (px > limit) px = limit;
    if (px < -limit) px = -limit;
    if (py > limit) py = limit;
    if (py < -limit) py = -limit;

    // 5. 计算四个轮组投影点到重力投影点P的距离 L0 ~ L3
    // 轮组坐标(X, Y)： LF(-W_L, W_L), LB(-W_L, -W_L), RB(W_L, -W_L), RF(W_L, W_L)
    float L0 = sqrtf((px - (-W_L)) * (px - (-W_L)) + (py - W_L) * (py - W_L));
    float L1 = sqrtf((px - (-W_L)) * (px - (-W_L)) + (py - (-W_L)) * (py - (-W_L)));
    float L2 = sqrtf((px - W_L) * (px - W_L) + (py - (-W_L)) * (py - (-W_L)));
    float L3 = sqrtf((px - W_L) * (px - W_L) + (py - W_L) * (py - W_L));

    // 6. 假设 N * L = 常数，计算支持力的权重系数 η0 ~ η3
    float inv_L0 = 1.0f / L0;
    float inv_L1 = 1.0f / L1;
    float inv_L2 = 1.0f / L2;
    float inv_L3 = 1.0f / L3;
    float sum_inv = inv_L0 + inv_L1 + inv_L2 + inv_L3;

    float eta0 = inv_L0 / sum_inv;
    float eta1 = inv_L1 / sum_inv;
    float eta2 = inv_L2 / sum_inv;
    float eta3 = inv_L3 / sum_inv;

    // 7. 计算低配版的总牵引力 f
    float beta_s = Get_Slope_Pitch(imu_data);
    float alpha = Get_Chassis_Yaw_Relative_To_Slope(imu_data);
    float f_magnitude = total_mass * GRAVITY_CONSTANT * sinf(beta_s);

    // 最好限制 f 大小范围 (防止疯车)
    if (fabsf(f_magnitude) > max_f)
    {
        f_magnitude = (f_magnitude > 0) ? max_f : -max_f;
    }

    Struct_Target_Force feedforward_force;
    feedforward_force.Fx = f_magnitude * sinf(alpha);
    feedforward_force.Fy = f_magnitude * cosf(alpha);
    feedforward_force.Mz = 0.0f;

    // 8. 动力学逆解算 (先得出低配版各轮平均分配的扭矩)
    Struct_Wheel_Torque low_end_torque;
    Omni_Wheel_Inverse_Dynamics(config, &feedforward_force, &low_end_torque);

    // 9. 牵引力重新分配 (高配版核心)
    // 注意：低配版的 Omni_Wheel_Inverse_Dynamics 函数中，四个轮子是平分总力的 (各占 1/4)
    // 高配版需要按照 eta 权重分配。为了保持总推力不变，我们要把原本的 1/4 替换成 eta
    // 即对低配版的扭矩乘上 (4.0 * eta)
    feedforward_torque->LF = low_end_torque.LF * (4.0f * eta0);
    feedforward_torque->LB = low_end_torque.LB * (4.0f * eta1);
    feedforward_torque->RB = low_end_torque.RB * (4.0f * eta2);
    feedforward_torque->RF = low_end_torque.RF * (4.0f * eta3);
}

/**
 * @brief 符号函数 sgn(x)，判断正负
 */
static float Math_Sign(float val)
{
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

/**
 * @brief 计算单轮低配版阻力补偿扭矩 (内部调用)
 *
 * @param omega 轮子目标角速度 (rad/s)
 * @param t 阻力补偿恒定扭矩 (N·m),需要提前测好，给电机很小的平动速度(如0.1rad/s)看电机的电流反馈值
 * @param cont_th 一阶连续化的角速度阈值 (rad/s);
 *                将 cont_th 设置为 1.0 rad/s,（电机抽搐 / 高频抖动）：说明在零点附近的前馈扭矩变化依然太剧烈,
 *                增大 cont_th（例如尝试 1.5 rad/s, 2.0 rad/s），让过渡区间更宽，力矩变化更平滑;
 *                (起步困难 / 响应迟钝)：如果在极低速指令下，电机“嗡嗡”响但不转，或者推动摇杆后底盘明显要“愣”一下才走,
 *                减小 cont_th（例如尝试 0.8 rad/s, 0.5 rad/s），让前馈扭矩更快、更陡峭地介入;
 * @return float 计算出的单轮阻力补偿扭矩 (N·m)
 */
static float Calc_Low_End_Single_Wheel(float omega, float t, float cont_th)
{
    float abs_omega = fabsf(omega);
    float torque = 0.0f;

    if (abs_omega <= cont_th && cont_th > 0.0f)
    {
        // 零点振荡：一阶连续化，用一段正比例函数贴合
        torque = t / cont_th * abs_omega;
    }
    else
    {
        // 正常恒定阻力补偿
        torque = t;
    }

    return torque * Math_Sign(omega);
}

/**
 * @brief 计算单轮高配版阻力补偿扭矩 (内部调用)
 *
 * @param omega 轮子目标角速度 (rad/s)
 * @param a0 二次多项式常数项
 * @param a1 二次多项式一次项系数
 * @param a2 二次多项式二次项系数
 *           分别是拟合阻力补偿曲线T = a0 + a1*|w| + a2*w^2 的常数项、一次项系数和二次项系数
 *           需通过下地实际测试和二阶最小二乘法计算得出。将底盘放在平地上（材质接近官方场地地胶）。给底盘若干个典型的平动目标速度
 *           （例如 0.1m/s, 0.2m/s, 0.5m/s, 0.8m/s, 1.2m/s, 1.5m/s, 1.8m/s, 2m/s等）。记录在每个目标速度下
 *            底盘匀速运动时各个电机的反馈电流值。利用数学软件（如 MATLAB、Excel 或 Python）对收集到的“反馈电流值 - 目标角速度”
 *            数据对进行二次最小二乘法回归分析，得出一条拟合曲线，直接获取对应的 a0, a1, a2 系数
 * @param cont_th 一阶连续化的角速度阈值 (rad/s)
*                 将 cont_th 设置为 1.0 rad/s,（电机抽搐 / 高频抖动）：说明在零点附近的前馈扭矩变化依然太剧烈,
 *                增大 cont_th（例如尝试 1.5 rad/s, 2.0 rad/s），让过渡区间更宽，力矩变化更平滑;
 *                (起步困难 / 响应迟钝)：如果在极低速指令下，电机“嗡嗡”响但不转，或者推动摇杆后底盘明显要“愣”一下才走,
 *                减小 cont_th（例如尝试 0.8 rad/s, 0.5 rad/s），让前馈扭矩更快、更陡峭地介入;
 * @param limit_th 前馈限幅的角速度阈值 (rad/s),需要评估你的电机最高工作转速，将其设置在一个合理的高速区间，确保前馈补偿不会超出电机的
 *                 最大承受电流或合理控制范围
 * @return float 计算出的单轮阻力补偿扭矩 (N·m)
 */
static float Calc_High_End_Single_Wheel(float omega, float a0, float a1, float a2, float cont_th, float limit_th)
{
    float abs_omega = fabsf(omega);
    float torque = 0.0f;

    if (abs_omega <= cont_th && cont_th > 0.0f)
    {
        // 零点振荡：一阶连续化
        // 1. 先算出连续化边界点 cont_th 处的前馈扭矩值
        float t_boundary = a0 + a1 * cont_th + a2 * cont_th * cont_th;
        // 2. 线性比例贴合
        torque = t_boundary / cont_th * abs_omega;
    }
    else if (abs_omega >= limit_th)
    {
        // 目标值太大发散：前馈限幅
        torque = a0 + a1 * limit_th + a2 * limit_th * limit_th;
    }
    else
    {
        // 正常二次拟合曲线段：T = a0 + a1*|w| + a2*w^2
        torque = a0 + a1 * abs_omega + a2 * abs_omega * abs_omega;
    }

    return torque * Math_Sign(omega);
}

/**
 * @brief 阻力扭矩补偿——低配版
 *
 * @param config 低配版阻力补偿物理参数
 * @param target_omega 底盘逆运动学解算出的四轮目标角速度
 * @param output_torque 解算后输出的四轮阻力补偿扭矩 (注意: 最终需乘以扭矩转电流系数才能输入给电机)
 */
void Calculate_Resistance_Compensation_Low_End(const Struct_Resistance_Config_Low* config,
                                                const Struct_Wheel_Velocity* target_omega,
                                                Struct_Wheel_Torque* output_torque)
{
    output_torque->LF = Calc_Low_End_Single_Wheel(target_omega->LF, config->t,
                      config->continuous_threshold);
    output_torque->LB = Calc_Low_End_Single_Wheel(target_omega->LB, config->t,
                      config->continuous_threshold);
    output_torque->RB = Calc_Low_End_Single_Wheel(target_omega->RB, config->t,
                      config->continuous_threshold);
    output_torque->RF = Calc_Low_End_Single_Wheel(target_omega->RF, config->t,
                      config->continuous_threshold);
}

/**
 * @brief 阻力扭矩补偿——高配版
 *
 * @param config 高配版阻力补偿曲线参数及限幅阈值
 * @param target_omega 底盘逆运动学解算出的四轮目标角速度
 * @param output_torque 解算后输出的四轮阻力补偿扭矩 (注意: 最终需乘以扭矩转电流系数才能输入给电机)
 */
void Calculate_Resistance_Compensation_High_End(const Struct_Resistance_Config_High* config,
                                                const Struct_Wheel_Velocity* target_omega,
                                                Struct_Wheel_Torque* output_torque)
{
    output_torque->LF = Calc_High_End_Single_Wheel(target_omega->LF, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
    output_torque->LB = Calc_High_End_Single_Wheel(target_omega->LB, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
    output_torque->RB = Calc_High_End_Single_Wheel(target_omega->RB, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
    output_torque->RF = Calc_High_End_Single_Wheel(target_omega->RF, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
}
