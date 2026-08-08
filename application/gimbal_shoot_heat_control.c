#include "gimbal_shoot_heat_control.h"
#include "referee.h"

#define HEAT_LIMIT 35

uint16_t heat0;
uint16_t heat0_limit;

uint8_t gimbal_shoot_heat_control(void)
{
	//获取裁判系统发射机构热量
	get_shoot_heat0_limit_and_heat0(&heat0_limit, &heat0);
	if(heat0_limit!=0)
	{
		if(heat0 >= heat0_limit - HEAT_LIMIT)
		{
			return 1;
		}
		else
			return 0;
	}
//	上裁判系统  2  下裁判系统  0
	return 0;
}
