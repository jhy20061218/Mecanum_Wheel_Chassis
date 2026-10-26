//
// Created by JHY on 2026/7/20.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_APP_KEYBOARD_CONTROL_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_APP_KEYBOARD_CONTROL_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief 按键状态枚举
 * 映射底层遥控器或键鼠解析后的按键动作
 */
typedef enum
{
    KEY_IDLE = 0,  // 按键空闲
    KEY_PRESSED,   // 按键按下
    KEY_HOLD,      // 按键保持
    KEY_RELEASED   // 按键释放
} Enum_Key_State;

/**
 * @brief 功率模式枚举
 */
typedef enum
{
    Power_Mode_NORMAL = 0,   // 常规模式
    Power_Mode_ACCELERATE,   // 加速模式
    Power_Mode_BURST         // 爆发模式
} Enum_Power_Mode;

/**
 * @brief 自瞄识别目标模式枚举
 */
typedef enum
{
    Auto_Aim_ARMOR = 0,      // 装甲板状态
    Auto_Aim_RUNE            // 能量机关状态
} Enum_Auto_Aim_Mode;

/**
 * @brief 队伍颜色枚举
 */
typedef enum
{
    Team_RED = 0,
    Team_BLUE
} Enum_Team_Color;

/**
 * @brief 优先级属性枚举 (用于底盘和发射机构)
 */
typedef enum
{
    Priority_HP = 0,         // 血量优先
    Priority_POWER,          // 功率优先
    Priority_COOLING,        // 冷却优先
    Priority_BURST           // 爆发优先
} Enum_Priority;

/**
 * @brief 底盘跟随模式枚举
 */
typedef enum
{
    Follow_Mode_NORMAL = 0,  // 底盘跟随云台
    Follow_Mode_OFF          // 底盘跟随关闭
} Enum_Follow_Mode;

/**
 * @brief 键鼠输入数据结构体
 * 将底层键鼠解算后的按键状态映射到此结构体
 */
typedef struct
{
    Enum_Key_State W;
    Enum_Key_State S;
    Enum_Key_State A;
    Enum_Key_State D;
    Enum_Key_State Q;
    Enum_Key_State E;
    Enum_Key_State Shift;
    Enum_Key_State Ctrl;
    Enum_Key_State B;
    Enum_Key_State V;
    Enum_Key_State C;
    Enum_Key_State X;
    Enum_Key_State Z;
    Enum_Key_State R;
    Enum_Key_State F;
} Struct_Keyboard_Data;

/**
 * @brief 系统综合状态结构体
 * 记录整车当前的状态机与全局配置参数
 */
typedef struct
{
    Enum_Power_Mode    power_mode;           // 功率模式 (常规/加速/爆发)
    Enum_Auto_Aim_Mode auto_aim_mode;        // 自瞄目标 (装甲板/能量机关)
    uint32_t           rune_state_timer;     // 能量机关状态计时器 (ms)

    uint8_t            super_cap_trusted;    // 超级电容可信标志位 (1:可信, 0:不可信)
    uint8_t            referee_trusted;      // 裁判系统可信标志位 (1:可信, 0:不可信)
    Enum_Team_Color    team_color;           // 队伍颜色 (红方/蓝方)
    Enum_Priority      chassis_priority;     // 底盘属性优先级 (血量优先/功率优先)
    Enum_Priority      shooter_priority;     // 发射机构属性优先级 (冷却优先/爆发优先)
    uint8_t            operator_level;       // 操作手手动设定等级 (1级/2级/3级)
} Struct_System_State;

/**
 * @brief 底盘运动目标指令结构体
 */
typedef struct
{
    float            vx;           // x轴(前后)平动期望速度 (m/s)
    float            vy;           // y轴(左右)平动期望速度 (m/s)
    float            omega;        // z轴自转期望角速度 (rad/s)
    Enum_Follow_Mode follow_mode;  // 底盘跟随状态 (跟随/不跟随)
} Struct_Chassis_Target;

/**
 * @brief 发射机构目标指令结构体
 */
typedef struct
{
    uint8_t is_firing_exceed_threshold;  // 是否突破连发阈值 (需由外部发射控制逻辑输入标记)
    float continuous_firing_threshold; // 当前模式下的连发限制阈值 (s)
} Struct_Shooter_Target;

void System_State_Init(Struct_System_State* sys_state);
void Keyboard_Strategy_Update(const Struct_Keyboard_Data* keyboard,
                              Struct_System_State* sys_state,
                              Struct_Chassis_Target* chassis_target,
                              Struct_Shooter_Target* shooter_target);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_APP_KEYBOARD_CONTROL_H
