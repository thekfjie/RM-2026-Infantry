#include "hatch.h"
#include "main.h"
#include "tim.h"

//初始化
void hatch_init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

//舱门无力
void hatch_no_move(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

//打开舱门
void hatch_open(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 700);	//舱门打开700
}
//关闭舱门
void hatch_close(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1700);	//舱门关闭1700
}


