//
// Created by 34285 on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_TIMER_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_TIMER_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief 状态枚举
 */
typedef enum
{
    Timer_Status_RESET = 0,
    Timer_Status_WAIT,
    Timer_Status_TRIGGER,
    Timer_Status_TIMEOUT,
} Enum_Timer_Status;

/**
 * @brief 定时器数据结构体
 */
typedef struct
{
    // 时钟计数
    uint32_t Tick;

    // 延迟时间单元数
    uint32_t Delay;

    // 当前状态
    Enum_Timer_Status Now_Status;
} Struct_Timer;

void Init_Timer(Struct_Timer *timer, uint32_t delay);
uint32_t Get_Timer_Tick(Struct_Timer *timer);
Enum_Timer_Status Get_Timer_Now_Status(Struct_Timer *timer);
void Set_Timer_Delay(Struct_Timer *timer, uint32_t delay);
void Timer_TIM_1ms_Calculate_PeriodElapsedCallback(Struct_Timer *timer);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_TIMER_H
