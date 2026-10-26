//
// Created by JHY on 2026/7/16.
//
#include "alg_queue.h"

/**
 * @brief 初始化循环队列，动态分配存储空间
 *
 * 为队列分配 max_size 个元素的内存空间（通过 calloc 清零初始化），
 * 并将队首(Front)、队尾(Rear)和当前长度(Length)归零。
 * queue 为空或 max_size 为 0 时安全返回。
 *
 * @param queue    队列结构体指针
 * @param max_size 队列最大容量（元素个数，必须 > 0）
 */
void Init_Queue(Struct_Queue *queue, uint32_t max_size)
{
    if (queue == NULL || max_size == 0) return;

    queue->Max_Size = max_size;
    queue->Front = 0;
    queue->Rear = 0;
    queue->Length = 0;
    queue->Data = (Queue_Data_Type *)calloc(max_size, sizeof(Queue_Data_Type));
}

/**
 * @brief 释放队列动态分配的内存
 *
 * 释放 Data 数组所占用的堆内存，并将 Max_Size 和 Length 归零，
 * 防止悬空指针和内存泄漏。queue 为空时安全返回。
 *
 * @param queue 队列结构体指针
 */
void Free_Queue(Struct_Queue *queue)
{
    if (queue != NULL)
    {
        if (queue->Data != NULL)
        {
            free(queue->Data);
            queue->Data = NULL;
        }
        queue->Max_Size = 0;
        queue->Length = 0;
    }
}

/**
 * @brief 获取队列当前元素个数
 *
 * @param queue 队列结构体指针
 * @return uint32_t 队列中已存储的元素个数，queue 为空时返回 0
 */
uint32_t Get_Queue_Length(Struct_Queue *queue)
{
    return (queue != NULL) ? queue->Length : 0;
}

/**
 * @brief 获取队首元素的值（不出队）
 *
 * 返回队列 Front 位置处的元素值，不会改变队列状态。
 * 队列为空、Data 为空或 queue 为空时返回 0。
 *
 * @param queue 队列结构体指针
 * @return Queue_Data_Type 队首元素的值
 */
Queue_Data_Type Get_Queue_Front(Struct_Queue *queue)
{
    if (queue == NULL || queue->Data == NULL || queue->Length == 0) return (Queue_Data_Type)0;
    return queue->Data[queue->Front];
}

/**
 * @brief 获取队尾元素的值（不出队）
 *
 * 返回队列 Rear 位置处的元素值（即最近一次 Push_Queue 写入的元素），
 * 不会改变队列状态。队列为空、Data 为空或 queue 为空时返回 0。
 *
 * @param queue 队列结构体指针
 * @return Queue_Data_Type 队尾元素的值
 */
Queue_Data_Type Get_Queue_Rear(Struct_Queue *queue)
{
    if (queue == NULL || queue->Data == NULL || queue->Length == 0) return (Queue_Data_Type)0;
    return queue->Data[queue->Rear];
}

/**
 * @brief 将元素入队（插入队尾）
 *
 * 将新元素 data 放入队列 Rear 的下一个位置（循环递增索引），同时 Length 加1。
 * 特殊处理：当队列为空（Length == 0）时，将 Front 和 Rear 都重置为 0。
 * 队列已满（Length == Max_Size）时不执行任何操作，防止溢出。
 *
 * @param queue 队列结构体指针
 * @param data  要入队的元素
 */
void Push_Queue(Struct_Queue *queue, Queue_Data_Type data)
{
    if (queue == NULL || queue->Data == NULL || queue->Length == queue->Max_Size)
    {
        return;
    }
    else
    {
        if (queue->Length == 0)
        {
            queue->Front = 0;
            queue->Rear = 0;
        }
        else
        {
            queue->Rear = (queue->Rear + 1) % queue->Max_Size;
        }

        queue->Data[queue->Rear] = data;
        queue->Length++;
    }
}

/**
 * @brief 将队首元素出队并返回其值
 *
 * 取出 Front 位置的元素值，然后将 Front 索引循环递增一位，Length 减1。
 * 队列为空、Data 为空或 queue 为空时返回 0。
 *
 * @param queue 队列结构体指针
 * @return Queue_Data_Type 被移除的队首元素值
 */
Queue_Data_Type Pop_Queue(Struct_Queue *queue)
{
    if (queue == NULL || queue->Data == NULL || queue->Length == 0)
    {
        return (Queue_Data_Type)0;
    }

    Queue_Data_Type temp = queue->Data[queue->Front];

    queue->Front = (queue->Front + 1) % queue->Max_Size;
    queue->Length--;

    return temp;
}

/**
 * @brief 清空队列所有元素
 *
 * 将 Front、Rear 和 Length 全部归零，逻辑上清空队列。
 * 注意：本函数不会释放 Data 内存，只是将队列状态重置为空。
 * 如需释放内存请调用 Free_Queue。
 *
 * @param queue 队列结构体指针
 */
void Clear_Queue(Struct_Queue *queue)
{
    if (queue != NULL)
    {
        queue->Front = 0;
        queue->Rear = 0;
        queue->Length = 0;
    }
}

