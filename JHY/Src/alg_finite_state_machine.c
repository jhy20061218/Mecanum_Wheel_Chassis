//
// Created by JHY on 2026/7/16.
//
#include "alg_finite_state_machine.h"

/**
 * @brief 状态机初始化
 *
 * 为有限状态机分配并重置状态数组，将所有状态标记为禁用(Status_Stage_DISABLE)并清零计时器，
 * 最后使能指定的初始状态。状态数量不能超过 STATUS_MAX。
 *
 * @param fsm               状态机结构体指针
 * @param status_number     状态总数（不能超过 STATUS_MAX）
 * @param now_status_serial 初始使能的状态序号（从0开始）
 */
void Init_FSM(Struct_FSM *fsm, uint8_t status_number, uint8_t now_status_serial)
{
    if (fsm == NULL || status_number > STATUS_MAX) return;

    fsm->Status_Number = status_number;
    fsm->Now_Status_Serial = now_status_serial;

    // 所有状态全刷0
    for (int i = 0; i < fsm->Status_Number; i++)
    {
        fsm->Status[i].Status_Stage = Status_Stage_DISABLE;
        fsm->Status[i].Count_Time = 0;
    }

    // 使能初始状态
    fsm->Status[now_status_serial].Status_Stage = Status_Stage_ENABLE;
}

/**
 * @brief 获取状态机当前激活的状态序号
 *
 * 返回状态机中当前处于使能状态(Status_Stage_ENABLE)的状态序号。
 * 若 fsm 为空指针则安全返回 0。
 *
 * @param fsm 状态机结构体指针
 * @return uint8_t 当前使能状态的序号，fsm 为空时返回 0
 */
uint8_t Get_FSM_Now_Status_Serial(Struct_FSM *fsm)
{
    return (fsm != NULL) ? fsm->Now_Status_Serial : 0;
}

/**
 * @brief 切换状态机的当前状态
 *
 * 先将当前状态的阶段设为禁用(Status_Stage_DISABLE)并清零其计时器，
 * 再将目标状态设为使能(Status_Stage_ENABLE)，最后将 Now_Status_Serial 更新为新的状态序号。
 * 若 fsm 为空或目标序号超出范围，则安全返回不做任何操作。
 *
 * @param fsm                 状态机结构体指针
 * @param next_status_serial  要切换到的目标状态序号（从0开始，必须小于 Status_Number）
 */
void Set_FSM_Status(Struct_FSM *fsm, uint8_t next_status_serial)
{
    if (fsm == NULL || next_status_serial >= fsm->Status_Number) return;

    // 失能当前状态, 计数器清零
    fsm->Status[fsm->Now_Status_Serial].Status_Stage = Status_Stage_DISABLE;
    fsm->Status[fsm->Now_Status_Serial].Count_Time = 0;

    // 转到下一个状态
    fsm->Status[next_status_serial].Status_Stage = Status_Stage_ENABLE;
    fsm->Now_Status_Serial = next_status_serial;
}

/**
 * @brief 状态机定时器周期回调：累加当前状态的运行计时
 *
 * 每次被定时器触发时，将当前使能状态的 Count_Time 计数器加1。
 * 该计数器用于记录当前状态已运行的时间单元数，具体的业务状态转移逻辑应
 * 在本函数外部基于 Count_Time 的值进行扩展编写。
 *
 * @param fsm 状态机结构体指针
 */
void FSM_TIM_Calculate_PeriodElapsedCallback(Struct_FSM *fsm)
{
    if (fsm == NULL) return;

    fsm->Status[fsm->Now_Status_Serial].Count_Time++;
    // 具体的业务状态转移逻辑在此函数外扩展调用或接着编写
}
