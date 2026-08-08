#include "user_lib.h"

fp32 arm_constrain(fp32 value, fp32 min, fp32 max)
{
    // 确保 min <= max（容错处理，避免参数传反）
    if (min > max)
    {
        fp32 temp = min;
        min = max;
        max = temp;
    }
    
    // 核心逻辑：如果值小于下限，返回下限；大于上限，返回上限；否则返回原值
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}