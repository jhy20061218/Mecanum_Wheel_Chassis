/* JHY: supervised experiment and telemetry interface. */
#ifndef APP_EXPERIMENT_H
#define APP_EXPERIMENT_H
#include "dvc_DJI_M3508.h"
#include "alg_mecanum_wheel_chassis.h"

enum {
    EXP_FAULT_OFFLINE = 1u, EXP_FAULT_TEMPERATURE = 2u,
    EXP_FAULT_OVERSPEED = 4u, EXP_FAULT_INVALID = 8u,
    EXP_FAULT_COMMAND_TIMEOUT = 16u, EXP_FAULT_ESTOP = 32u,
    EXP_FAULT_CAN = 64u
};
void Experiment_Init(Struct_RM_Motor_M3508 *lf, Struct_RM_Motor_M3508 *lb,
                     Struct_RM_Motor_M3508 *rb, Struct_RM_Motor_M3508 *rf,
                     Struct_Chassis_Config *config);
void Experiment_Feedback(unsigned wheel);
void Experiment_Control_1ms(void);
void Experiment_Poll(void);
void Experiment_CAN_Tx_Result(HAL_StatusTypeDef result);
#endif
