//
// Created by 34285 on 2026/6/4.
//

#ifndef MOTOR_DJI_M3508_H
#define MOTOR_DJI_M3508_H

#include "stm32f4xx_hal.h"
#include "RM_motor_common.h"
#include "alg_PID.h"

/**
 * @brief M3508对应的C620电调结构体
 */
typedef struct
{
    Struct_PID PID_Angle;
    Struct_PID PID_Omega;

    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_RM_Motor_ID RM_Motor_ID;
    //电机接收缓冲区
    uint8_t *CAN_Tx_Data;
    //减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
    float Gearbox_Rate;
    //最大扭矩
    float Torque_Max;

    //电机转一圈编码器的刻度
    uint16_t Encoder_Num_Per_Round;
    //输出的最大值
    uint16_t Output_Max;

    //电机状态的标志位
    uint32_t Flag;
    //电机上一次状态的标志位
    uint32_t Pre_Flag;
    //电调反馈的电机机械角度
    uint16_t Rx_Encoder;
    //电调反馈的电机转速
    int16_t Rx_Omega;
    //电调反馈的电机扭矩
    int16_t Rx_Torque;
    //电调反馈的电机当前温度
    uint16_t Rx_Temperature;
    //上一次电机的编码器值
    uint16_t Pre_Encoder;
    //电机总共的编码器值
    int32_t Total_Encoder;
    //电机总共的圈数
    int32_t Total_Round;

    Enum_RM_Motor_Status RM_Motor_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    uint8_t Now_Temperature;

    Enum_Control_Method Control_Method;
    float Target_Angle;
    float Target_Omega;
    float Target_Torque;
    float Out;
}Struct_RM_Motor_M3508;

void DJI_M3508_Init(CAN_HandleTypeDef *hcan, Struct_RM_Motor_M3508 *motor, Enum_RM_Motor_ID motor_id,
                    Enum_Control_Method control_method, float gearbox_rate, float torque_max);
void DJI_M3508_Output(Struct_RM_Motor_M3508 *motor);

uint16_t DJI_M3508_Get_Output_Max(Struct_RM_Motor_M3508 *motor);
Enum_RM_Motor_Status DJI_M3508_Get_CAN_Motor_Status(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Now_Angle(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Now_Omega(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Now_Torque(Struct_RM_Motor_M3508 *motor);
uint8_t DJI_M3508_Get_Now_Temperature(Struct_RM_Motor_M3508 *motor);
Enum_Control_Method DJI_M3508_Get_Control_Method(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Target_Angle(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Target_Omega(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Target_Torque(Struct_RM_Motor_M3508 *motor);
float DJI_M3508_Get_Out(Struct_RM_Motor_M3508 *motor);

void DJI_M3508_Set_Control_Method(Struct_RM_Motor_M3508 *motor, Enum_Control_Method control_method);
void DJI_M3508_Set_Target_Angle(Struct_RM_Motor_M3508 *motor, float target_angle);
void DJI_M3508_Set_Target_Omega(Struct_RM_Motor_M3508 *motor, float target_omega);
void DJI_M3508_Set_Target_Torque(Struct_RM_Motor_M3508 *motor, float target_torque);
void DJI_M3508_Set_Out(Struct_RM_Motor_M3508 *motor, float out);

void DJI_M3508_CAN_RxCpltCallback(Struct_RM_Motor_M3508 *motor, uint8_t *Rx_Data);
void DJI_M3508_TIM_Alive_PeriodElapsedCallback(Struct_RM_Motor_M3508 *motor);
void DJI_M3508_TIM_PID_PeriodElapsedCallback(Struct_RM_Motor_M3508 *motor);

#endif //MOTOR_DJI_M3508_H
