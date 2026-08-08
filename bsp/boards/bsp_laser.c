#include "bsp_laser.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

void laser_init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
