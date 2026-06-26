//
// Created by 34285 on 2026/4/19.
//

#ifndef DRV_MATH_H
#define DRV_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <float.h>

/* Exported macros -----------------------------------------------------------*/

//圆周率PI
#define PI 3.14159265358979323846f

/**
 * @brief 限幅宏 (替代C++的Template)
 * * @param x_ptr 传入数据的指针
 * @param min 最小值
 * @param max 最大值
 */
#define MATH_CONSTRAIN(x_ptr, min, max) \
do {                                \
if (*(x_ptr) < (min)) {         \
*(x_ptr) = (min);           \
} else if (*(x_ptr) > (max)) {  \
*(x_ptr) = (max);           \
}                               \
} while (0)

/**
 * @brief 求绝对值宏 (替代C++的Template)
 * * @param x 传入数据
 */
#define MATH_ABS(x) (((x) > 0) ? (x) : -(x))


/* Exported function declarations --------------------------------------------*/

/* C语言不支持函数重载，修改了带源/目标地址参数的函数名称 */
void Math_Endian_Reverse_16(void *Address);
void Math_Endian_Reverse_16_Copy(void *Source, void *Destination);
void Math_Endian_Reverse_32(void *Address);
void Math_Endian_Reverse_32_Copy(void *Source, void *Destination);

uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

float Math_Sinc(float x);

#ifdef __cplusplus
}
#endif

#endif

