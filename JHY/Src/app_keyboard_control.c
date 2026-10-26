//
// Created by JHY on 2026/7/20.
//
#include "app_keyboard_control.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

/**
 * @brief 系统状态初始化，配置默认状态
 *
 * 对功率、超级电容、裁判系统、目标识别等功能设定默认值
 *
 * @param sys_state 系统综合状态结构体指针
 */
void System_State_Init(Struct_System_State* sys_state)
{
    if (sys_state == NULL) return;

    // 功率相关：默认常规模式
    sys_state->power_mode = Power_Mode_NORMAL;

    // 能量机关：默认装甲板状态
    sys_state->auto_aim_mode = Auto_Aim_ARMOR;
    sys_state->rune_state_timer = 0; // 能量机关状态计时器清零

    // 模块与整车控制：默认状态
    sys_state->super_cap_trusted = 1;     // 超级电容可信：默认状态
    sys_state->referee_trusted = 1;       // 裁判系统可信：默认状态
    sys_state->team_color = Team_RED;     // 红蓝方：默认红方
    sys_state->chassis_priority = Priority_HP;       // 底盘属性：默认血量优先
    sys_state->shooter_priority = Priority_COOLING;  // 发射机构属性：默认冷却优先
    sys_state->operator_level = 1;        // 操作手手动升级：默认1级
}

/**
 * @brief 键鼠控制策略核心状态机与逻辑更新
 *
 * 将键鼠输入转化为底盘、云台、发射机构的控制目标指令
 *
 * @param keyboard      键鼠输入数据结构体 (包含按键按下 PRESSED 和按住 HOLD 的状态)
 * @param sys_state     系统综合状态结构体指针 (需持续维持状态)
 * @param chassis_target 底盘运动目标指令结构体
 * @param shooter_target 发射机构目标指令结构体
 */
void Keyboard_Strategy_Update(const Struct_Keyboard_Data* keyboard,
                              Struct_System_State* sys_state,
                              Struct_Chassis_Target* chassis_target,
                              Struct_Shooter_Target* shooter_target)
{
    if (keyboard == NULL || sys_state == NULL || chassis_target == NULL || shooter_target == NULL)
    {
        return;
    }

    // ==============================================================
    // 1.模块与整车控制逻辑 (组合键 Ctrl + Shift + 任意键)
    // ==============================================================
    uint8_t is_ctrl_shift_held = (keyboard->Ctrl == KEY_HOLD && keyboard->Shift == KEY_HOLD);

    if (is_ctrl_shift_held)
    {
        // 超级电容可信情况切换, Ctrl+Shift+C【按下】
        if (keyboard->C == KEY_PRESSED)
        {
            sys_state->super_cap_trusted = !sys_state->super_cap_trusted;
        }

        // 红蓝方切换, Ctrl+Shift+Q【按下】
        if (keyboard->Q == KEY_PRESSED)
        {
            sys_state->team_color = (sys_state->team_color == Team_RED) ? Team_BLUE : Team_RED;
            sys_state->referee_trusted = 0; // 手动修改导致裁判系统不可信
        }

        // 底盘属性切换, Ctrl+Shift+W【按下】
        if (keyboard->W == KEY_PRESSED)
        {
            sys_state->chassis_priority = (sys_state->chassis_priority == Priority_HP) ? Priority_POWER : Priority_HP;
            sys_state->referee_trusted = 0;
        }

        // 发射机构属性切换, Ctrl+Shift+E【按下】
        if (keyboard->E == KEY_PRESSED)
        {
            sys_state->shooter_priority = (sys_state->shooter_priority == Priority_COOLING) ?
                                           Priority_BURST : Priority_COOLING;
            sys_state->referee_trusted = 0;
        }

        // 操作手手动升级/降级, Ctrl+Shift+R/F【按下】
        if (keyboard->R == KEY_PRESSED)
        {
            if (sys_state->operator_level < 3) sys_state->operator_level++;
            sys_state->referee_trusted = 0;
        }
        if (keyboard->F == KEY_PRESSED)
        {
            if (sys_state->operator_level > 1) sys_state->operator_level--;
            sys_state->referee_trusted = 0;
        }

        // 复位整车, Ctrl+Shift+Z【按下】
        if (keyboard->Z == KEY_PRESSED)
        {
            // 调用 HAL 库复位函数脱困
            // HAL_NVIC_SystemReset();
        }

        // 裁判系统可信情况切换, Ctrl+Shift+X【按下】
        if (keyboard->X == KEY_PRESSED)
        {
            // 需手动切回才能继续相信
            sys_state->referee_trusted = 1;
        }
    }

    // ==============================================================
    // 2.功率相关与底盘机动性能限制设定
    // ==============================================================
    // 功率模式判断：Shift / B 【按住】
    if (keyboard->B == KEY_HOLD)
    {
        // 无论是否按 Shift，B按住即优先进入爆发模式
        sys_state->power_mode = Power_Mode_BURST;
    }
    else if (keyboard->Shift == KEY_HOLD && !is_ctrl_shift_held) // 防止与组合键冲突
    {
        sys_state->power_mode = Power_Mode_ACCELERATE;
    }
    else
    {
        sys_state->power_mode = Power_Mode_NORMAL;
    }

    // 设定底盘机动性能上限
    float limit_vmax = 3.0f;
    float limit_wmax = 4.0f * PI;
    float limit_acc  = 3.0f; // 留作斜坡函数(Slope Planner)参数设定使用

    if (sys_state->power_mode == Power_Mode_NORMAL)
    {
        limit_vmax = 3.0f;
        limit_wmax = 4.0f * PI;
        limit_acc  = 3.0f;
    }
    else if (sys_state->power_mode == Power_Mode_ACCELERATE || sys_state->power_mode == Power_Mode_BURST)
    {
        // 加速模式与爆发模式机动性上限一致，主要区别在限制功率阈值设定
        limit_vmax = 5.0f;
        limit_wmax = 6.0f * PI;
        limit_acc  = 5.0f;
    }

    // ==============================================================
    // 3.键鼠逻辑——底盘运动控制
    // ==============================================================
    chassis_target->vx = 0.0f;
    chassis_target->vy = 0.0f;
    chassis_target->omega = 0.0f;

    // 当不处于全局配置组合键状态下时，才响应平动与自转
    if (!is_ctrl_shift_held)
    {
        // 平动控制 (WSAD 按住) - 第一人称视角
        if (keyboard->W == KEY_HOLD) chassis_target->vx += limit_vmax;
        if (keyboard->S == KEY_HOLD) chassis_target->vx -= limit_vmax;
        if (keyboard->A == KEY_HOLD) chassis_target->vy += limit_vmax;
        if (keyboard->D == KEY_HOLD) chassis_target->vy -= limit_vmax;

        // 旋转控制 - 小陀螺 (QE 按住)
        if (keyboard->Q == KEY_HOLD) chassis_target->omega += limit_wmax;
        if (keyboard->E == KEY_HOLD) chassis_target->omega -= limit_wmax;
    }

    // ==============================================================
    // 4.能量机关——自瞄识别的目标切换
    // ==============================================================
    // 自瞄目标切换, V【按下】
    if (keyboard->V == KEY_PRESSED)
    {
        if (sys_state->auto_aim_mode == Auto_Aim_ARMOR)
        {
            sys_state->auto_aim_mode = Auto_Aim_RUNE;
            sys_state->rune_state_timer = 0; // 重置进入打符状态的时间
        }
        else
        {
            sys_state->auto_aim_mode = Auto_Aim_ARMOR;
        }
    }

    // 能量机关状态的强制退出逻辑判定
    if (sys_state->auto_aim_mode == Auto_Aim_RUNE)
    {
        // 定时器累加 (假设该函数调用频率为 1kHz, dt = 1ms)
        sys_state->rune_state_timer += 1;

        uint8_t force_exit = 0;

        // 条件1: 开启了小陀螺 (Q或E按住, omega目标值不为0)
        if (chassis_target->omega != 0.0f)
        {
            force_exit = 1;
        }
        // 条件2: 持续超过30s (30000 ms)
        else if (sys_state->rune_state_timer > 30000)
        {
            force_exit = 1;
        }
        // 条件3: 突破了发射机构连发阈值 (这里通过外部传感器或判断逻辑给出)
        else if (shooter_target->is_firing_exceed_threshold)
        {
            force_exit = 1;
        }

        // 满足强制退出条件，自动切回装甲板状态
        if (force_exit)
        {
            sys_state->auto_aim_mode = Auto_Aim_ARMOR;
        }
    }

    // 根据模式下发跟随与射击阈值参数
    if (sys_state->auto_aim_mode == Auto_Aim_ARMOR)
    {
        chassis_target->follow_mode = Follow_Mode_NORMAL;
        shooter_target->continuous_firing_threshold = 0.15f;
    }
    else // Auto_Aim_RUNE自瞄模式
    {
        chassis_target->follow_mode = Follow_Mode_OFF; // 能量机关时不跟随
        shooter_target->continuous_firing_threshold = 1.0f;
    }
}