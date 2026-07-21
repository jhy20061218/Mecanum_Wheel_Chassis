//
// Created by 34285 on 2026/7/16.
//
#include "alg_timer.h"

/**
 * @brief 初始化定时器
 *
 * @param timer 目标定时器结构体指针
 * @param delay 延迟最大幅度
 */
void Init_Timer(Struct_Timer *timer, uint32_t delay)
{
    if (timer == NULL)
    {
        return;
    }

    timer->Delay = delay;
    timer->Tick = 0;
    timer->Now_Status = Timer_Status_RESET;
}

/**
 * @brief 获取时钟计数
 *
 * @param timer 目标定时器结构体指针
 * @return uint32_t 时钟计数
 */
uint32_t Get_Timer_Tick(Struct_Timer *timer)
{
    if (timer == NULL)
    {
        return 0;
    }
    return timer->Tick;
}

/**
 * @brief 获取当前状态
 *
 * @param timer 目标定时器结构体指针
 * @return Enum_Timer_Status 当前状态
 */
Enum_Timer_Status Get_Timer_Now_Status(Struct_Timer *timer)
{
    if (timer == NULL)
    {
        return Timer_Status_RESET;
    }
    return timer->Now_Status;
}

/**
 * @brief 设定延时参数并刷新当前状态
 *
 * @param timer 目标定时器结构体指针
 * @param delay 延迟时间单元数
 */
void Set_Timer_Delay(Struct_Timer *timer, uint32_t delay)
{
    if (timer == NULL)
    {
        return;
    }

    timer->Delay = delay;
    timer->Tick = 0;

    if (timer->Delay != 0)
    {
        timer->Now_Status = Timer_Status_WAIT;
    }
    else
    {
        timer->Now_Status = Timer_Status_RESET;
    }
}

/**
 * @brief 定时器调整状态, 计时周期1ms
 *
 * @param timer 目标定时器结构体指针
 */
void Timer_TIM_1ms_Calculate_PeriodElapsedCallback(Struct_Timer *timer)
{
    if (timer == NULL)
    {
        return;
    }

    timer->Tick++;

    if (timer->Delay == 0)
    {
        timer->Now_Status = Timer_Status_RESET;
    }
    else if (timer->Tick < timer->Delay)
    {
        timer->Now_Status = Timer_Status_WAIT;
    }
    else if (timer->Tick == timer->Delay)
    {
        timer->Now_Status = Timer_Status_TRIGGER;
    }
    else if (timer->Tick > timer->Delay)
    {
        timer->Now_Status = Timer_Status_TIMEOUT;
    }
}

