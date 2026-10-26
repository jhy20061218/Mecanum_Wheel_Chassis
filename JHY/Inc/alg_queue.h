//
// Created by JHY on 2026/7/16.
//

#ifndef ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_QUEUE_H
#define ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_QUEUE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// 可以自由修改这里的类型，例如 typedef int Queue_Data_Type;
typedef float Queue_Data_Type;

/**
 * @brief 循环队列结构体
 */
typedef struct
{
    // 队列本体数组 (动态分配)
    Queue_Data_Type *Data;

    // 队列最大容量
    uint32_t Max_Size;
    // 队首元素位置
    uint32_t Front;
    // 队尾元素位置
    uint32_t Rear;
    // 队列当前长度
    uint32_t Length;
} Struct_Queue;

void Init_Queue(Struct_Queue *queue, uint32_t max_size);
void Free_Queue(Struct_Queue *queue);
uint32_t Get_Queue_Length(Struct_Queue *queue);
Queue_Data_Type Get_Queue_Front(Struct_Queue *queue);
Queue_Data_Type Get_Queue_Rear(Struct_Queue *queue);
void Push_Queue(Struct_Queue *queue, Queue_Data_Type data);
Queue_Data_Type Pop_Queue(Struct_Queue *queue);
void Clear_Queue(Struct_Queue *queue);

#endif //ALL_DIRECTIONAL_WHEEL_CHASSIS_ALG_QUEUE_H
