#include "shoot.h"

#define TRIGGER_ANGLE_KP		8.0f
#define TRIGGER_ANGLE_KI		0.0f
#define TRIGGER_ANGLE_KD		0.0f
#define TRIGGER_ANGLE_MAX_OUT	10000.0f
#define TRIGGER_ANGLE_MAX_IOUT	0.0f
//防堵转电流阈值
#define TRIGGER_CURRENT_THRE	8500.0f

shoot_task_t shoot;

trigger_detect_e trigger_detect_task(void)
{
	if(shoot.trigger_ecd != 0)
		return TRIGGER_ON_LINE;
	else
		return TRIGGER_OFF_LINE;
}

void shoot_task(uint8_t *trigger_state_out, uint8_t shoot_mode, uint8_t fric_mode, fp32 *speed, fp32 shoot_delay)
{
	shoot_feedback(&shoot);
	Motor_Angle_Cal(&shoot, TRIGGER_RATIO);
	static uint16_t i;
	//标志位用来判断是否按下鼠标
	static uint8_t trigger_add_state = 0;
	if(shoot_mode != SHOOT_STOP&&fric_mode == FRIC_START)
	{
		if(trigger_add_state == 0)
		{
			if(gimbal_shoot_heat_control() == 0||shoot_mode == SHOOT_WITHOUT_CONTROL)
				shoot.angle_set += TRIGGER_OUT_VAL;
			trigger_add_state = 1;
		}
	}
	if(trigger_add_state == 1)
		i++;
	if(i >= shoot_delay)
	{
		trigger_add_state = 0;
		i = 0;
	}
	//堵转判断
	shoot.trigger_state = trigger_prevent_stall(&shoot.angle_set, shoot.shoot_trigger_t.shoot_motor_measure->given_current, shoot.shoot_trigger_t.speed);
	*speed = PID_calc(&shoot.trigger_angle_pid, shoot.angle.POS_ABS, shoot.angle_set);
	*trigger_state_out = shoot.trigger_state;
}


void shoot_init(void)
{
	shoot.shoot_trigger_t.shoot_motor_measure = get_shoot_motor_measure_point(0);
	shoot.angle_set = START_POS;
	const static fp32 trigger_angle_pid[3] = {TRIGGER_ANGLE_KP, TRIGGER_ANGLE_KI, TRIGGER_ANGLE_KD};
	PID_init(&shoot.trigger_angle_pid, PID_POSITION, trigger_angle_pid, TRIGGER_ANGLE_MAX_OUT, TRIGGER_ANGLE_MAX_IOUT);
}

void shoot_feedback(shoot_task_t *shoot_feedback_point)
{
	shoot_feedback_point->shoot_trigger_t.relative_angle = shoot_feedback_point->shoot_trigger_t.shoot_motor_measure->ecd;
	//转角度制
	shoot_feedback_point->real_angle = shoot_feedback_point->shoot_trigger_t.relative_angle/8192.0f*360.0f;
	shoot_feedback_point->trigger_ecd = shoot_feedback_point->shoot_trigger_t.shoot_motor_measure->ecd;
	shoot_feedback_point->shoot_trigger_t.speed = shoot_feedback_point->shoot_trigger_t.shoot_motor_measure->speed_rpm;
}

uint8_t trigger_prevent_stall(fp32 *angle, fp32 current, fp32 speed)
{
	if(current > TRIGGER_CURRENT_THRE&&speed>0)
	{
		*angle -= TRIGGER_OUT_VAL*2;
		return TRIGGER_BLOCK_TURN;
	}
	else if(current < -TRIGGER_CURRENT_THRE)
	{
		*angle += TRIGGER_OUT_VAL;
		return TRIGGER_BLOCK_TURN;
	}
	return TRIGGER_TURN;
}

void Motor_Angle_Cal(shoot_task_t *shoot_motor_point, float T)
{
	float  res1, res2;
//	int  res3, res4;
	static float pos, pos_old;
	
	pos = shoot_motor_point->real_angle;
	shoot_motor_point->angle.eer=pos - pos_old;

	if(shoot_motor_point->angle.eer>0)
	{
		res1=shoot_motor_point->angle.eer-T;//反转，自减
		res2=shoot_motor_point->angle.eer;
	}
	else
	{
		res1=shoot_motor_point->angle.eer+T;//正转，自加一个周期的角度值（360）
		res2=shoot_motor_point->angle.eer;
	}

	if(fabs(res1)<fabs(res2)) //不管正反转，肯定是转的角度小的那个是真的
	{
		shoot_motor_point->angle.eer_eer = res1;
	}
	else
	{
		shoot_motor_point->angle.eer_eer = res2;
	}

	shoot_motor_point->angle.POS_ABS += shoot_motor_point->angle.eer_eer;
	pos_old  = pos;
}

