//
// Created by JHY on 2026/7/20.
//
#include "drv_wdg.h"

/**
 * @brief 喂独立看门狗
 * 注意，看门狗不可以放在TIM中断回调函数中使用
 */
void IWDG_Independent_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
}
