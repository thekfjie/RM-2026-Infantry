/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled∆¡ƒªœ‘ æ¥ÌŒÛ¬Î
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef OLED_TASK_H
#define OLED_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "chassis_task.h"



typedef struct
{
	const RC_ctrl_t *oled_rc_ctrl;
	const gimbal_control_t *oled_gimbal_point;
	const chassis_move_t *oled_chassis_point;
}oled_task_t;

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled»ŒŒÒ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void oled_task(void const * argument);


#endif
