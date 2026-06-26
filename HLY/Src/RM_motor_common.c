//
// Created by 34285 on 2026/6/4.

#include "RM_motor_common.h"

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

//CAN通信发送缓冲区
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x1FF_Tx_Data[8];
uint8_t CAN1_0x2FF_Tx_Data[8];
uint8_t CAN1_0x1FE_Tx_Data[8];
uint8_t CAN1_0x2FE_Tx_Data[8];

uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x1FF_Tx_Data[8];
uint8_t CAN2_0x2FF_Tx_Data[8];
uint8_t CAN2_0x1FE_Tx_Data[8];
uint8_t CAN2_0x2FE_Tx_Data[8];

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param Motor_ID CAN ID
 * @return 缓冲区指针
 */
uint8_t *Allocate_Tx_Data(CAN_HandleTypeDef *hcan, Enum_RM_Motor_ID Motor_ID)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan->Instance == CAN1)
    {
        switch (Motor_ID)
        {
            case (RM_Motor_ID_0x201):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
                }
            break;
            case (RM_Motor_ID_0x202):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
                }
            break;
            case (RM_Motor_ID_0x203):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
                }
            break;
            case (RM_Motor_ID_0x204):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
                }
            break;
            case (RM_Motor_ID_0x205):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[0]);
                }
            break;
            case (RM_Motor_ID_0x206):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[2]);
                }
            break;
            case (RM_Motor_ID_0x207):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[4]);
                }
            break;
            case (RM_Motor_ID_0x208):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[6]);
                }
            break;
            case (RM_Motor_ID_0x209):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FE_Tx_Data[0]);
                }
            break;
            case (RM_Motor_ID_0x20A):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FE_Tx_Data[2]);
                }
            break;
            case (RM_Motor_ID_0x20B):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FE_Tx_Data[4]);
                }
            break;
            default:
            break;
        }
    }
    else if (hcan->Instance == CAN2)
    {
        switch (Motor_ID)
        {
        case (RM_Motor_ID_0x201):
            {
                tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
            }
            break;
        case (RM_Motor_ID_0x202):
            {
                tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
            }
            break;
        case (RM_Motor_ID_0x203):
            {
                tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
            }
            break;
        case (RM_Motor_ID_0x204):
            {
                tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
            }
            break;
        case (RM_Motor_ID_0x205):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[0]);
            }
            break;
        case (RM_Motor_ID_0x206):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[2]);
            }
            break;
        case (RM_Motor_ID_0x207):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[4]);
            }
            break;
        case (RM_Motor_ID_0x208):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[6]);
            }
            break;
        case (RM_Motor_ID_0x209):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FE_Tx_Data[0]);
            }
            break;
        case (RM_Motor_ID_0x20A):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FE_Tx_Data[2]);
            }
            break;
        case (RM_Motor_ID_0x20B):
            {
                tmp_tx_data_ptr = &(CAN2_0x1FE_Tx_Data[4]);
            }
            break;
        default:
            break;
        }
    }
    return tmp_tx_data_ptr;
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE,
                               0x200, 0x7F0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE,
                               0x200, 0x7F0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE,
                               0x200, 0x7F0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE,
                               0x200, 0x7F0);
    }
}

/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param Filter_ID ID
 * @param Mask_ID 屏蔽位
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t Filter_ID, uint32_t Mask_ID)
{
    //自定义滤波器结构体名称
    CAN_FilterTypeDef can_filter_mask_structure;

    //通过第0位ID，判断是数据帧或遥控帧，遥控帧不处理
    if (Object_Para & 0x01)
    {
        return;
    }

    //通过第1位ID，判断是标准帧或扩展帧，扩展帧不处理
    if ((Object_Para & 0x02) >> 1)
    {
        return;
    }

    can_filter_mask_structure.FilterIdHigh = (Filter_ID & 0x7FF) << 5;
    can_filter_mask_structure.FilterIdLow = 0x0000;
    can_filter_mask_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
    can_filter_mask_structure.FilterMaskIdLow = 0x0000;
    can_filter_mask_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    can_filter_mask_structure.FilterBank = (Object_Para >> 3) & 0x1F;
    can_filter_mask_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_mask_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_mask_structure.FilterActivation = ENABLE;
    can_filter_mask_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_mask_structure);
}

/**
 * @brief 发送CAN数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;
    tx_header.TransmitGlobalTime = DISABLE ;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox);
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback(void)
{
    // CAN1总线电机
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    // CAN_Send_Data(&hcan1, 0x1FF, CAN1_0x1FF_Tx_Data, 8);
    // CAN_Send_Data(&hcan1, 0x2FF, CAN1_0x2FF_Tx_Data, 8);
    // CAN_Send_Data(&hcan1, 0x1FE, CAN1_0x1FE_Tx_Data, 8);
    // CAN_Send_Data(&hcan1, 0x2FE, CAN1_0x2FE_Tx_Data, 8);

    // CAN2总线电机
    // CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x1FF, CAN2_0x1FF_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x2FF, CAN2_0x2FF_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x1FE, CAN2_0x1FE_Tx_Data, 8);
    // CAN_Send_Data(&hcan2, 0x2FE, CAN2_0x2FE_Tx_Data, 8);
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header,
                             CAN1_Manage_Object.Rx_Buffer.Data);
        if (CAN1_Manage_Object.Callback_Function != NULL)
        {
            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header,
                             CAN2_Manage_Object.Rx_Buffer.Data);
        if (CAN2_Manage_Object.Callback_Function != NULL)
        {
            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header,
                             CAN1_Manage_Object.Rx_Buffer.Data);
        if (CAN1_Manage_Object.Callback_Function != NULL)
        {
            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header,
                             CAN2_Manage_Object.Rx_Buffer.Data);
        if (CAN2_Manage_Object.Callback_Function != NULL)
        {
            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
    }
}
