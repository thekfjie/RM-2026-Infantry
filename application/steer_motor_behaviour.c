#include "steer_motor_behaviour.h"

uint16_t steer_motor_ecd_behaviour_task(uint16_t now_ecd, uint16_t limit_ecd)
{
	uint16_t out;
	if(now_ecd > limit_ecd)
	{
		out = now_ecd - limit_ecd;
	}
	else
	{
		out = now_ecd;
	}
	return out;
}

