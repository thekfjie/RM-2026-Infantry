#ifndef _AIMBOT_SOLVING
#define _AIMBOT_SOLVING

#include "main.h"
#include "struct_typedef.h"
#include "math.h"
#include "AHRS_MiddleWare.h"


typedef enum
{
	X,
	Y,
	Z,
}coordinate_e;

uint8_t aimbot_get_robot_id(void);
uint8_t Get_ID(void);
fp32 aimbot_yaw_world_to_rad(float x, float y, float z);
fp32 aimbot_pitch_world_to_rad(float x, float y, float z);
fp32 aimbot_gravity_compensation(float v0, float pitch, float t);
fp32 projectile_solve(float distance, float z0, float t);
fp32 low_pass_filter(float input, float alpha) ;
#endif
