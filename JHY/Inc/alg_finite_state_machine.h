//
// Created by JHY on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_FINITE_STATE_MACHINE_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_FINITE_STATE_MACHINE_H

#include <stdint.h>
#include <stddef.h>

#define STATUS_MAX (10)

/**
 * @brief 状态所处的阶段
 */
typedef enum
{
    Status_Stage_DISABLE = 0,
    Status_Stage_ENABLE,
} Enum_Status_Stage;

/**
 * @brief 状态结构体
 */
typedef struct
{
    Enum_Status_Stage Status_Stage;
    uint32_t Count_Time;
} Struct_Status;

/**
 * @brief 有限自动机核心结构体
 */
typedef struct
{
    Struct_Status Status[STATUS_MAX];
    // 状态数量
    uint8_t Status_Number;
    // FSM当前状态
    uint8_t Now_Status_Serial;
} Struct_FSM;

void Init_FSM(Struct_FSM *fsm, uint8_t status_number, uint8_t now_status_serial);
uint8_t Get_FSM_Now_Status_Serial(Struct_FSM *fsm);
void Set_FSM_Status(Struct_FSM *fsm, uint8_t next_status_serial);
void FSM_TIM_Calculate_PeriodElapsedCallback(Struct_FSM *fsm);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_FINITE_STATE_MACHINE_H
