#include "aimbot_solving.h"
#include "referee.h"

#define GRAVITY_G	9.7833f
#define ARRAY_SIZE 100

uint8_t aimbot_get_robot_id(void)
{
	if(get_robot_id() <= 7)
		return 1;
	else
		return 0;
}
uint8_t Get_ID(void)
{
	uint8_t id = get_robot_id();
	if(id==3){
		return 1;
	}
	else if(id==4){
		return 1;
	}
	else if(id==5){
		return 1;
	}
	else if(id == 7){
		return 1;
	}
	else if(id==103){
		return 0;
	}
	else if(id==104){
		return 0;
	}
	else if(id==105){
		return 0;
	}
	else if(id == 107)
		return 0;
	return 0;
}

fp32 aimbot_yaw_world_to_rad(float x, float y, float z)
{
	fp32 yaw_rad;
	yaw_rad = atan(y/x); 

	if(x<0 && y>0)
		yaw_rad += PI;
	else if(x<0 && y<0)
		yaw_rad -= PI;
	
	return yaw_rad;
}

fp32 aimbot_pitch_world_to_rad(float x, float y, float z)
{
	fp32 pitch_rad;
	fp32 xy = sqrt(x*x+y*y);
	
	pitch_rad = atanf(z/xy);

	
	return pitch_rad;
}

//zÖáÖØÁ¦²¹³¥
fp32 projectile_solve(float distance, float z0, float t)
{
	float zg_buff, z_total;
	zg_buff = GRAVITY_G*t*t/2;
	z_total = zg_buff*0.20f + z0;
	
	return z_total;
}

//Ò»½×µÍÍ¨ÂË²¨
fp32 low_pass_filter(float input, float alpha) {
	fp32 filtered_output;
	static fp32 last_filtered;
	
    filtered_output = alpha * input + (1.0f - alpha) * last_filtered;
    last_filtered = input;
    return filtered_output;
}




