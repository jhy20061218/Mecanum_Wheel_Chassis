//
// Created by 34285 on 2026/7/14.
//
#include "alg_power_management.h"

/**
 * @brief 初始化功率控制算法句柄
 *
 * @param power_ctrl 功率控制句柄指针
 * @param est_config 功率估计参数结构体指针
 * @param pid_kp 功率PID比例系数
 * @param pid_ki 功率PID积分系数
 * @param pid_kd 功率PID微分系数
 * @param pid_dt PID运算周期（秒）
 */
void Power_Control_Init(Struct_Power_Control* power_ctrl,
                        const Struct_Power_Estimate_Config* est_config,
                        float pid_kp, float pid_ki, float pid_kd, float pid_dt)
{
    // 拷贝功率估计参数
    power_ctrl->estimate_config = *est_config;

    // 初始化内部的PID控制器 (无前馈，限幅视具体情况而定，这里给个充裕的输出限幅)
    PID_Init(&(power_ctrl->power_pid), pid_kp, pid_ki, pid_kd, 0.0f,
             50.0f, 100.0f, pid_dt, 0.0f, 0.0f, 0.0f,
             0.0f, PID_D_First_Disable);
}

/**
 * @brief 底盘单电机功率估计值计算
 *
 * 基于电机的当前电流和速度值估算其消耗的电功率
 *
 * @param config 功率估计参数
 * @param state 电机当前状态（电流和角速度）
 * @return float 估计的单电机功率 (单位: W)
 */
float Estimate_Single_Motor_Power(const Struct_Power_Estimate_Config* config,
                                  const Struct_Motor_State* state)
{
    // 机械功率项 (正比于 转速 * 扭矩电流 的绝对值，考虑发电机效应则可能需要分象限讨论，这里取最稳妥的绝对值累加)
    float mech_power = config->k1 * fabsf(state->omega * state->current);

    // 热损耗项 (正比于电流的平方，I^2 * R)
    float heat_power = config->k2 * state->current * state->current;

    // 总估计功率
    float total_estimated_power = mech_power + heat_power + config->p0;

    return total_estimated_power;
}

/**
 * @brief 计算底盘电机整体功率重分配的衰减因数
 *
 * 1. 经过 PID 计算功率调整量
 * 2. 结合裁判系统限制功率，得出底盘目标总功率
 * 3. 估计当前底盘各电机的总功率需求
 * 4. 功率重分配算法得出 0~1 的衰减因数
 *
 * @param power_ctrl          功率控制综合句柄
 * @param max_power           最大功率（作为PID的 Target）
 * @param current_power       当前功率（作为PID的 Now）
 * @param referee_limit_power 裁判系统限制功率
 * @param motors_state        底盘四个电机当前的电流和速度值
 * @return float              底盘各电机功率衰减因数 (范围: 0.0 ~ 1.0)
 */
float Calculate_Power_Attenuation_Factor(Struct_Power_Control* power_ctrl,
                                         float max_power,
                                         float current_power,
                                         float referee_limit_power,
                                         const Struct_Chassis_Motors_State* motors_state)
{
    // ==============================================================
    // 1. PID 模块：计算功率补偿量
    // ==============================================================
    // 设定目标值与当前值
    PID_Set_Target(&(power_ctrl->power_pid), max_power);
    PID_Set_Now(&(power_ctrl->power_pid), current_power);

    // 执行PID计算周期
    PID_TIM_Adjust_PeriodElapsedCallback(&(power_ctrl->power_pid));

    // 获取PID输出
    float pid_out = PID_Get_Out(&(power_ctrl->power_pid));

    // ==============================================================
    // 2. 加法器 (+)：计算底盘允许的目标总功率
    // ==============================================================
    // PID输出 + 裁判系统限制功率
    float allowed_total_power = pid_out + referee_limit_power;

    // 兜底防护：允许输出的总功率不能低于0
    if (allowed_total_power < 0.0f)
    {
        allowed_total_power = 0.0f;
    }

    // ==============================================================
    // 3. 功率估计模块：底盘各电机功率估计值计算
    // ==============================================================
    float est_p_LF = Estimate_Single_Motor_Power(&(power_ctrl->estimate_config), &(motors_state->LF));
    float est_p_LB = Estimate_Single_Motor_Power(&(power_ctrl->estimate_config), &(motors_state->LB));
    float est_p_RB = Estimate_Single_Motor_Power(&(power_ctrl->estimate_config), &(motors_state->RB));
    float est_p_RF = Estimate_Single_Motor_Power(&(power_ctrl->estimate_config), &(motors_state->RF));

    // 预测的底盘总需求功率
    float estimated_total_power = est_p_LF + est_p_LB + est_p_RB + est_p_RF;

    // ==============================================================
    // 4. 底盘电机整体功率重分配算法 (输出 0~1 的衰减因数)
    // ==============================================================
    float scale_factor = 1.0f; // 默认不衰减

    // 如果预测所需总功率 大于 允许的目标总功率，则需要按比例衰减
    if (estimated_total_power > allowed_total_power)
    {
        // 防止除数为0的容错处理
        if (estimated_total_power > 0.001f)
        {
            scale_factor = allowed_total_power / estimated_total_power;
        }
        else
        {
            scale_factor = 0.0f;
        }
    }

    // 限幅防护，严格保证输出范围在 [0.0, 1.0] 之间
    if (scale_factor > 1.0f)
    {
        scale_factor = 1.0f;
    }
    else if (scale_factor < 0.0f)
    {
        scale_factor = 0.0f;
    }

    return scale_factor;
}

/**
 * @brief 初始化基于 Maclaurin 展开的功率控制算法句柄
 *
 * @param power_ctrl 功率控制句柄指针
 * @param est_config 功率估计参数结构体指针
 * @param pid_kp 功率PID比例系数
 * @param pid_ki 功率PID积分系数
 * @param pid_kd 功率PID微分系数
 * @param pid_dt PID运算周期（秒）
 */
void Maclaurin_Power_Control_Init(Struct_Maclaurin_Power_Control* power_ctrl,
                                  const Struct_Maclaurin_Estimate_Config* est_config,
                                  float pid_kp, float pid_ki, float pid_kd, float pid_dt)
{
    // 拷贝 Maclaurin 功率估计参数 (k0 ~ k5)
    power_ctrl->estimate_config = *est_config;

    // 初始化内部的PID控制器 (无前馈，限幅视具体情况而定，这里给个充裕的输出限幅)
    PID_Init(&(power_ctrl->power_pid), pid_kp, pid_ki, pid_kd, 0.0f,
             50.0f, 100.0f, pid_dt, 0.0f, 0.0f, 0.0f,
             0.0f, PID_D_First_Disable);
}

/**
 * @brief 底盘单电机功率估计值计算 (基于 Maclaurin 展开)
 *
 * 基于理论课件中的二元函数二阶 Maclaurin 展开，利用相电流和本体角速度估算功率。
 * 公式：P = k0 + k1 * I + k2 * ω + k3 * I * ω + k4 * I^2 + k5 * ω^2
 * 注意：角速度应当是不带减速箱的本体角速度值，且 I 是电机反馈回来的相电流。
 *
 * @param config 功率估计模型配置参数 (由最小二乘法离线拟合得出)
 * @param state 电机当前状态（电流和角速度）
 * @return float 估计的单电机功率 (单位: W)
 */
float Estimate_Single_Motor_Power_Maclaurin(const Struct_Maclaurin_Estimate_Config* config,
                                            const Struct_Motor_State* state)
{
    float I = state->current;
    float w = state->omega;

    // 代入多元函数展开的估计公式
    float estimated_power = config->k0
                          + config->k1 * I
                          + config->k2 * w
                          + config->k3 * I * w
                          + config->k4 * I * I
                          + config->k5 * w * w;

    return estimated_power;
}

/**
 * @brief 计算底盘电机整体功率重分配的衰减因数 (Maclaurin 模型版)
 *
 * 1. 经过 PID 计算功率调整量
 * 2. 结合裁判系统限制功率，得出底盘目标总功率
 * 3. 根据 Maclaurin 模型，估计当前底盘各电机的总功率需求
 * 4. 功率重分配算法得出 0~1 的衰减因数
 *
 * @param power_ctrl          功率控制综合句柄
 * @param max_power           最大功率（作为PID的 Target）
 * @param current_power       当前功率（作为PID的 Now）
 * @param referee_limit_power 裁判系统限制功率
 * @param motors_state        底盘四个电机当前的电流和速度值
 * @return float              底盘各电机功率衰减因数 (范围: 0.0 ~ 1.0)
 */
float Calculate_Maclaurin_Power_Attenuation(Struct_Maclaurin_Power_Control* power_ctrl,
                                            float max_power,
                                            float current_power,
                                            float referee_limit_power,
                                            const Struct_Chassis_Motors_State* motors_state)
{
    // ==============================================================
    // 1. PID 模块：计算功率补偿量
    // ==============================================================
    PID_Set_Target(&(power_ctrl->power_pid), max_power);
    PID_Set_Now(&(power_ctrl->power_pid), current_power);

    // 执行PID计算周期
    PID_TIM_Adjust_PeriodElapsedCallback(&(power_ctrl->power_pid));
    float pid_out = PID_Get_Out(&(power_ctrl->power_pid));

    // ==============================================================
    // 2. 加法器 (+)：计算底盘允许的目标总功率
    // ==============================================================
    float allowed_total_power = pid_out + referee_limit_power;

    // 兜底防护：允许输出的总功率不能低于0
    if (allowed_total_power < 0.0f)
    {
        allowed_total_power = 0.0f;
    }

    // ==============================================================
    // 3. 功率估计模块：底盘各电机功率估计值计算 (6参数模型)
    // ==============================================================
    float est_p_LF = Estimate_Single_Motor_Power_Maclaurin(&(power_ctrl->estimate_config), &(motors_state->LF));
    float est_p_LB = Estimate_Single_Motor_Power_Maclaurin(&(power_ctrl->estimate_config), &(motors_state->LB));
    float est_p_RB = Estimate_Single_Motor_Power_Maclaurin(&(power_ctrl->estimate_config), &(motors_state->RB));
    float est_p_RF = Estimate_Single_Motor_Power_Maclaurin(&(power_ctrl->estimate_config), &(motors_state->RF));

    // 预测的底盘总需求功率
    float estimated_total_power = est_p_LF + est_p_LB + est_p_RB + est_p_RF;

    // ==============================================================
    // 4. 底盘电机整体功率重分配算法 (输出 0~1 的衰减因数)
    // ==============================================================
    float scale_factor = 1.0f; // 默认不衰减

    // 如果预测所需总功率 大于 允许的目标总功率，则需要按比例衰减
    if (estimated_total_power > allowed_total_power)
    {
        // 防止除数为0的容错处理
        if (estimated_total_power > 0.001f)
        {
            scale_factor = allowed_total_power / estimated_total_power;
        }
        else
        {
            scale_factor = 0.0f;
        }
    }

    // 限幅防护，严格保证输出范围在 [0.0, 1.0] 之间
    if (scale_factor > 1.0f)
    {
        scale_factor = 1.0f;
    }
    else if (scale_factor < 0.0f)
    {
        scale_factor = 0.0f;
    }

    return scale_factor;
}

/**
* @brief 解一元二次方程 a*x^2 + b*x + c = 0,并筛选 [0, 1] 范围内的有效解
*
* @param a 二次项系数
* @param b 一次项系数
* @param c 常数项
* @return float 返回有效的衰减系数,若无解或不合理则返回 0.0f
*/
static float Math_Solve_Eta(float a, float b, float c)
{
    // 1. 退化为一元一次方程的情况
    if (fabsf(a) < 1e-6f)
    {
        if (fabsf(b) < 1e-6f) return 0.0f;
        float eta = -c / b;
        if (eta >= 0.0f && eta <= 1.0f) return eta;
        return 0.0f;
    }

    // 2. 计算判别式 delta
    float delta = b * b - 4.0f * a * c;
    if (delta < 0.0f)
    {
        return 0.0f; // 无解,说明电机未工作在合理范围,直接返回0限制输出
    }

    // 3. 求两根
    float sqrt_delta = sqrtf(delta);
    float eta1 = (-b + sqrt_delta) / (2.0f * a);
    float eta2 = (-b - sqrt_delta) / (2.0f * a);

    // 4. 判断有效解,要求在 [0, 1] 内
    float valid_eta = 0.0f;
    int valid_count = 0;
    if (eta1 >= 0.0f && eta1 <= 1.0f)
    {
        valid_eta = eta1;
        valid_count++;
    }
    if (eta2 >= 0.0f && eta2 <= 1.0f)
    {
        if (valid_count == 0)
        {
            valid_eta = eta2;
            valid_count++;
        }
        else
        {

            // 如果有两个解都在 0~1 之间,取较大的那个
            valid_eta = (eta1 > eta2) ? eta1 : eta2;
            valid_count++;
        }
    }
    return valid_eta;
}

/**
* @brief 衰减电流法(标准):整体衰减所有电流项 (Maclaurin 模型)
*
* 单电机功率的 Maclaurin 展开公式:
*   P_motor = k0 + k1*I + k2*w + k3*I*w + k4*I^2 + k5*w^2
* 引入衰减系数 eta 后,单电机功率关于 eta 的一元二次方程:
*   P_motor(eta) = A*eta^2 + B*eta + C
*   其中: 二次项系数 A = k4 * I^2
*         一次项系数 B = k1 * I + k3 * I * w
*         常数项   C = k0 + k2 * w + k5 * w^2
* 联立方程 A*eta^2 + B*eta + (C - P_target) = 0,求解 eta ∈ [0, 1]
*
* @param power_ctrl 功率控制结构体指针
* @param max_power 最大功率限制
* @param current_power 当前功率
* @param referee_limit_power 裁判系统限制功率
* @param motors_state 底盘电机状态结构体指针
* @return float 返回衰减系数
*/
float Calculate_Maclaurin_Current_Attenuation(Struct_Maclaurin_Power_Control* power_ctrl,
                                              float max_power,
                                              float current_power,
                                              float referee_limit_power,
                                              const Struct_Chassis_Motors_State* motors_state)
{
    // 1. PID 模块:计算功率补偿量
    PID_Set_Target(&(power_ctrl->power_pid), max_power);
    PID_Set_Now(&(power_ctrl->power_pid), current_power);
    PID_TIM_Adjust_PeriodElapsedCallback(&(power_ctrl->power_pid));
    float pid_out = PID_Get_Out(&(power_ctrl->power_pid));

    // 2. 加法器:计算底盘允许的目标总功率
    float allowed_total_power = pid_out + referee_limit_power;
    if(allowed_total_power < 0.0f)
    {
        allowed_total_power = 0.0f;
    }
    const Struct_Maclaurin_Estimate_Config* cfg = &(power_ctrl->estimate_config);
    float A_sum = 0.0f, B_sum = 0.0f, C_sum = 0.0f;
    const Struct_Motor_State* motors[4] =
        {
            &(motors_state->LF),
            &(motors_state->LB),
            &(motors_state->RB),
            &(motors_state->RF)
        };

    // 3. 遍历四个电机,累加一元二次方程的系数
    for (int i = 0; i < 4; ++i)
    {
        float I = motors[i]->current;
        float w = motors[i]->omega;
        A_sum += cfg->k4 * I * I;
        B_sum += cfg->k1 * I + cfg->k3 * I * w;
        C_sum += cfg->k0 + cfg->k2 * w + cfg->k5 * w * w;
    }

    // 4. 判断原始功率是否超限
    float p_origin = A_sum + B_sum + C_sum;
    if (p_origin <= allowed_total_power)
    {
        return 1.0f; // 不超限,直接原样下发
    }

    // 5. 超限求解 A*eta^2 + B*eta + C = P_max
    return Math_Solve_Eta(A_sum, B_sum, C_sum - allowed_total_power);
}

/**
* @brief 衰减电流法(特殊情况):单独衰减动力学解算部分,保留其他补偿部分,适合极限工况（如重装上坡且功率受限）
*
* @param power_ctrl 功率控制结构体指针
* @param max_power 最大功率限制
* @param current_power 当前功率
* @param referee_limit_power 裁判系统限制功率
* @param split_motors_state 分离式底盘电机状态结构体指针
* @return float 返回衰减系数
*/
float Calculate_Maclaurin_Special_Current_Attenuation(Struct_Maclaurin_Power_Control* power_ctrl,
                                                      float max_power,
                                                      float current_power,
                                                      float referee_limit_power,
                                                      const Struct_Chassis_Split_Motors_State* split_motors_state)
{
    // 1. PID 模块:计算功率补偿量
    PID_Set_Target(&(power_ctrl->power_pid), max_power);
    PID_Set_Now(&(power_ctrl->power_pid), current_power);
    PID_TIM_Adjust_PeriodElapsedCallback(&(power_ctrl->power_pid));
    float pid_out = PID_Get_Out(&(power_ctrl->power_pid));

    // 2. 加法器:计算底盘允许的目标总功率
    float allowed_total_power = pid_out + referee_limit_power;
    if (allowed_total_power < 0.0f)
    {
        allowed_total_power = 0.0f;
    }
    const Struct_Maclaurin_Estimate_Config* cfg = &(power_ctrl->estimate_config);
    float A_sum = 0.0f, B_sum = 0.0f, C_sum = 0.0f;
    const Struct_Split_Motor_State* motors[4] =
        {
            &(split_motors_state->LF), &(split_motors_state->LB),
            &(split_motors_state->RB), &(split_motors_state->RF)
        };

    // 3. 遍历四个电机,累加分离式一元二次方程的系数
    for (int i = 0; i < 4; ++i)
    {
        float Id = motors[i]->dynamic_current;
        float Io = motors[i]->others_current;
        float w = motors[i]->omega;
        // 展开合并同类项
        float Ai = cfg->k4 * Id * Id;
        float Bi = cfg->k1 * Id + cfg->k3 * Id * w + 2.0f * cfg->k4 * Id * Io;
        float Ci = cfg->k0 + cfg->k1 * Io + cfg->k2 * w + cfg->k3 * Io * w + cfg->k4 * Io * Io + cfg->k5 * w * w;
        A_sum += Ai;
        B_sum += Bi;
        C_sum += Ci;
    }

    // 4. 判断原始功率是否超限 (此时设 eta = 1)
    float p_origin = A_sum + B_sum + C_sum;
    if (p_origin <= allowed_total_power)
    {
        return 1.0f; // 不超限,直接原样下发
    }

    // 5. 超限求解 A*eta^2 + B*eta + C = P_max
    return Math_Solve_Eta(A_sum, B_sum, C_sum - allowed_total_power);
}

