#ifndef __USER_LIB_H
#define __USER_LIB_H

#include "stdint.h"
#include "math.h"


typedef float fp32;

/**
 * @brief  浮点型数值限幅函数
 * @param  value: 需要限幅的数值
 * @param  min:   最小值（下限）
 * @param  max:   最大值（上限）
 * @retval 限幅后的数值
 */
fp32 arm_constrain(fp32 value, fp32 min, fp32 max);

#endif
