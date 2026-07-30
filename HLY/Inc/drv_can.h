//
// Created by 34285 on 2026/6/4.
//

#ifndef RM_MOTOR_COMMON_H
#define RM_MOTOR_COMMON_H

#include "stm32f4xx_hal.h"
#include "drv_math.h"

#define RPM_TO_RADPS (2.0f * PI / 60.0f)

//滤波器编号
#define CAN_FILTER(x) ((x) << 3)

//接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

//数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/**
 * @brief 电机状态枚举类型
 *
 */
typedef enum
{
    RM_Motor_Status_Disable = 0,
    RM_Motor_Status_Enable
}Enum_RM_Motor_Status;

/**
 * @brief 电机ID枚举类型
 * @attention M3508:201~204, M2006:205~208, GM6020:209~20B, 也可根据需求重新定义
 *
 */
typedef enum
{
    RM_Motor_ID_UNDEFINED = 0,
    RM_Motor_ID_0x201,
    RM_Motor_ID_0x202,
    RM_Motor_ID_0x203,
    RM_Motor_ID_0x204,
    RM_Motor_ID_0x205,
    RM_Motor_ID_0x206,
    RM_Motor_ID_0x207,
    RM_Motor_ID_0x208,
    RM_Motor_ID_0x209,
    RM_Motor_ID_0x20A,
    RM_Motor_ID_0x20B
}Enum_RM_Motor_ID;

/**
 * @brief CAN电机的ID分配情况
 *
 */
typedef enum
{
    RM_Motor_ID_Status_FREE = 0,
    RM_Motor_ID_Status_ALLOCATED
}Enum_RM_Motor_ID_Allocated_Status;

/**
 * @brief 电机控制方式
 *
 */
typedef enum
{
    Control_Method_OPENLOOP = 0,
    Control_Method_VOLTAGE,
    Control_Method_CURRENT,
    Control_Method_TORQUE,
    Control_Method_OMEGA,
    Control_Method_ANGLE
}Enum_Control_Method;

/**
 * @brief CAN接收的信息结构体
 *
 */
typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
}Struct_CAN_Rx_Buffer;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
}Struct_CAN_Manage_Object;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

uint8_t *Allocate_Tx_Data(CAN_HandleTypeDef *hcan, Enum_RM_Motor_ID Motor_ID);
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t Filter_ID, uint32_t Mask_ID);
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
void TIM_CAN_PeriodElapsedCallback(void);

#endif //RM_MOTOR_COMMON_H
