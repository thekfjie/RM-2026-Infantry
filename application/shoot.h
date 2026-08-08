#ifndef _SHOOT_H
#define _SHOOT_H

#include "main.h"
#include "pid.h"
#include "can_receive.h"
#include "math.h"
#include "gimbal_shoot_heat_control.h"

//周期
#define TRIGGER_RATIO	360.0f	//360/8
//每个拨弹出弹丸需要走的编码器值
#define TRIGGER_OUT_VAL	1440.0f			//1440.0f
//起始位置
#define	START_POS		0.0f
//堵转标志位
typedef enum
{
	TRIGGER_BLOCK_TURN,		//堵了
	TRIGGER_TURN,			//没堵
}trigger_state_e;

typedef struct
{
	const motor_measure_t *shoot_motor_measure;
	fp32 count;
	fp32 accel;
	fp32 accel_set;
	fp32 speed;
	fp32 speed_set;
	
	fp32 shoot_delay;
	
	fp32 current_set;
	fp32 motor_gyro_set;
	uint32_t target_ecd;
	fp32 relative_angle;
	fp32 relative_angle_set;
	int16_t give_current;
} shoot_motor_t;

//遥控器编码轮
typedef struct
{
	uint8_t shoot_state;	//用来处理f键换模式
	uint8_t shoot_mode;
	uint8_t fric_mode;
	
	uint8_t trigger_state;
	uint8_t fric_flag;
	
	fp32 encoding_switch_ecd;
	
}encoding_switch_t;

typedef enum
{
	SHOOT_SINGLE_START,
	SHOOT_CONTINUOUS_START,
	SHOOT_CONTROL,
	SHOOT_WITHOUT_CONTROL,
	SHOOT_SINGLE_START_STATE,
	SHOOT_CONTINUOUS_START_STATE,
	SHOOT_WITHOUT_CONTROL_STATE,
	SHOOT_STOP,
	SHOOT_OUT,
	
	FRIC_READY,
	FRIC_STOP,
	FRIC_START,
	
}shoot_mode_e;

typedef enum
{
	TRIGGER_ON_LINE,
	TRIGGER_OFF_LINE,
}trigger_detect_e;
//一些角度控制的结构体变量
typedef struct
{
	float POS_GAOL;//目标位置
	float POS_ABS;//绝对位置0
	float POS_OFFSET;
	float eer;
	float eer_eer;
}angle_typedef;


typedef struct
{
	shoot_motor_t shoot_trigger_t;
	angle_typedef angle;
	fp32 angle_set;			//trigger发射角度制设定
	fp32 real_angle;
	uint8_t trigger_ecd;
	pid_type_def trigger_speed_pid;
	pid_type_def trigger_angle_pid;
	
	uint8_t trigger_state;
}shoot_task_t;

void shoot_init(void);
uint8_t trigger_prevent_stall(fp32 *angle, fp32 current, fp32 speed);
void shoot_task(uint8_t *trigger_state_out, uint8_t trigger_mode, uint8_t fric_mode, fp32 *speed, fp32 shoot_delay);
void Motor_Angle_Cal(shoot_task_t *shoot_motor_point, float T);
void shoot_feedback(shoot_task_t *shoot_feedback_point);

#endif
