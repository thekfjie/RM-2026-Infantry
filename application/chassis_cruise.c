#include "chassis_cruise.h"
#include "cmsis_os.h"

cruise_control_t cruise_control;

void chassis_cruise_task(void const *pvParameters)
{
    chassis_cruise_init(&cruise_control);
	while(1)
	{
	//	chassis_cruise_feedback_update(&cruise_control);
		chassis_cruise_set_mode(&cruise_control);
		chassis_cruise_control_loop(&cruise_control);
	
	}
}

void chassis_cruise_init(cruise_control_t *chassis_cruise_init_t)
{
//	chassis_cruise_init_t->chassis_cruise_point = get_chassis_move_point();
//	chassis_cruise_init_t->gimbal_cruise_point = get_gimbal_move_point();
}

void chassis_cruise_set_mode(cruise_control_t *chassis_cruise_mode_t)
{
	//获取云台模式
//	chassis_cruise_mode_t->cruise_mode = chassis_cruise_mode_t->chassis_cruise_point->chassis_mode;
}

void chassis_cruise_feedback_update(cruise_control_t *chassis_cruise_feedback_update_t)
{
//	chassis_cruise_feedback_update_t->absolute_angle = chassis_cruise_feedback_update_t->chassis_cruise_point->chassis_motor->absolute_angle*ABSOLUTE_MOVE_TATE;
//	if(chassis_cruise_feedback_update_t->absolute_angle>8000)
}

void chassis_cruise_control_loop(cruise_control_t *chassis_cruise_control_loop_t)
{
//	if(chassis_cruise_control_loop_t->cruise_mode == CHASSIS_NO_MOVE)
//	{
//		chassis_cruise_control_loop_t->move_vx = 0;
//		chassis_cruise_control_loop_t->move_vy = 0;
//		chassis_cruise_control_loop_t->move_wz = 0;
//	}
//	else if(chassis_cruise_control_loop_t->cruise_mode == CHASSIS_CRUISE_NORMAL)
//	{
//		chassis_cruise_control_loop_t->yaw = 1200;
//		chassis_cruise_control_loop_t->pitch = 3;
		chassis_cruise_control_loop_t->move_vy = 2000;
		vTaskDelay(4000);
//		chassis_cruise_control_loop_t->pitch = -3;
		chassis_cruise_control_loop_t->move_vy = -2000;
		vTaskDelay(4000);
//		HAL_Delay(1000);
//	}
}

/**
  * @brief          获取云台数据指针
  * @param[in]      none
  * @retval         获取云台数据指针
  */
const cruise_control_t *get_chassis_cruise_point(void)
{
    return &cruise_control;
}

