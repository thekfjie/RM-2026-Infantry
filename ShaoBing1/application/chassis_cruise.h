#ifndef _CRUISE_TASK_H
#define _CRUISE_TASK_H

#include "main.h"
#include "struct_typedef.h"
//#include "chassis_task.h"
//#include "gimbal_task.h"

#define ABSOLUTE_MOVE_TATE 0.0024220749f	//120PI/(19*8192) µ¥Î»cm

typedef struct
{
//	const gimbal_control_t *gimbal_cruise_point;
//	const chassis_move_t *chassis_cruise_point;
	
	uint8_t cruise_mode;
		
	uint16_t absolute_angle;
	
	fp32 yaw;
	fp32 pitch;
	
	fp32 move_x;
	fp32 move_y;
	fp32 move_yaw;
	
	fp32 move_vx;
	fp32 move_vy;
	fp32 move_wz;
}cruise_control_t;

extern void chassis_cruise_init(cruise_control_t *chassis_cruise_init_t);
extern void chassis_cruise_set_mode(cruise_control_t *chassis_cruise_mode_t);
extern void chassis_cruise_feedback_update(cruise_control_t *chassis_cruise_feedback_update_t);
extern void chassis_cruise_control_loop(cruise_control_t *chassis_cruise_control_loop_t);
extern void chassis_cruise_task(void const *pvParameters);
extern const cruise_control_t *get_chassis_cruise_point(void);

#endif
