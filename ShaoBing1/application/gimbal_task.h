#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "aimbot_task.h"
#include "chassis_cruise.h"
#include "referee.h"
#include "shoot.h"

#define ABS(x) (((x)>0)?(x):(-(x)))

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 500

#define GIMBAL_CONTROL_TIME 1
//小陀螺云台yaw轴补偿
#define GIMBAL_SPIN_YAW_BUFFER	0.0005f
//拨弹轮速度
#define TRIGGER_SHOOT_SPEED_RATE_FAST	80	
#define TRIGGER_SHOOT_SPEED_RATE_SLOW 	130
#define TRIGGER_SHOOT_STOP 0
//拨弹轮电机和轮盘转换比例
#define TRIGGER_TO_TURN_RATE 45		//8/36(轮盘孔径个数/点击减速比)
//面对哨兵，自己的左右2
#define FRIC_SHOOT_LEFT_SPEED 7000
			//MAX 8000
#define FRIC_SHOOT_RIGHT_SPEED	7000



////云台转动幅度
//#define GIMBAL_TURA_RATE 20.0f
//底盘跟随云台模式下，底盘转动限幅
#define GIMBAO_TURN_LIMIT 850
//底盘跟随模式下舵轮触发灵敏度
#define GIMBAL_CHASSIS_STEER_SEN 240
//遥控器 灵敏度
#define GIMBAL_OPEN_RC_YAW_SEN		3.0f		//影响编码器底盘跟随模式下的最大偏转角度
#define GIMBAL_OPEN_RC_PITCH_SEN		1.0f		//0.0012f
#define GIMBAO_OPEN_RC_PITCH_FAST_SEN	1.70f		//0.0024f
//陀螺仪PITCH轴转换比例
#define GIMBAL_PITCH_ECD_RATE			0.0018f
#define GIMBAL_PITCH_INS_RATE			0.0000013f
//陀螺仪yaw轴灵敏度
#define YAW_INS_OPEN_RC_YAW_SEN		0.000002f			//陀螺仪模式灵敏度
//鼠标	灵敏度
#define GIMBAL_OPEN_X_YAW_SEN		27.0f
#define GIMBAL_OPEN_Y_PITCH_SEN		27.0f
//在chassis_open 模型下，自瞄乘以该比例发送到can上
#define AIMBOT_OPEN_YAW_SCALE     800.0f
#define AIMBOT_OPEN_PITCH_SCALE  3.0f
//yaw,pitch控制通道以及状态开关通道
#define RC_YAW_CHANNEL   0
#define RC_PITCH_CHANNEL 1
#define SHOOT_MODE_CHANNEL 4

//yaw轴起始位置
#define M6020_GIMBAL_ANGLE_YAW			   2413    //1341     //5418 		//6808


#define M6020_GIMBAL_ANGLE_PITCH	4400
#define PITCH_ECD_MAX				5124
#define PITCH_ECD_MIN				3900
#define PITCH_INS_MAX				0.5f
#define PITCH_INS_MIN				0.25f

#define M2006_GIMBAL_ANGLE_TRIGGER			0		
#define M2006_GIMBAL_ANGLE_TRIGGER_TURN		1024



typedef enum
{
	REMOTE_CONTROL,
	KEYBOARD_CONTROL,
}control_methods_e;

typedef enum
{
	HATCH_OPEN,
	HATCH_CLOSE,
	HATCH_STAY,
}hatch_state_e;


typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  uint16_t offset_ecd;
  fp32 max_relative_angle; //rad
  fp32 min_relative_angle; //rad
	
	fp32 add;
  fp32 relative_angle;     //rad
  fp32 relative_angle_set; //rad
  fp32 absolute_angle;     //rad
  fp32 absolute_angle_set; //加上初始化angle后的输入
  fp32 motor_gyro;         //rad/s
  fp32 motor_gyro_set;
  fp32 current_angle;
  fp32 last_angle;
  fp32 target_ecd;
  fp32 angle_set;			//传入
  fp32 err_angle;
  fp32 err_angle_set; //rad
  fp32 accle;        		//rad/s
  fp32 accle_set;
  fp32 speed;
  fp32 speed_set;
  fp32 raw_cmd_current;
  fp32 current_set;
  int16_t give_current;

} gimbal_motor_t;


//陀螺仪数据
typedef struct
{
	//小陀螺
	fp32 yaw_target;
	fp32 pitch_target;
	
	fp32 roll;
	fp32 pitch;
	fp32 yaw;
	fp32 yaw_last;
	
	fp32 out;
	fp32 add;	//累加
	fp32 err;
}ins_data_t;

typedef struct
{
	const RC_ctrl_t *gimbal_rc_ctrl;
	const aimbot_task_t *gimbal_aimbot_point;
	const cruise_control_t *gimbal_cruise_point;
	const fp32 *gimbal_INT_angle_point;
	const fp32 *gimbal_INT_gyro_point;
	
	fp32 chassis_power_gimbal;
	fp32 shoot_heat_gimbal;
	
//	aimbot_count_t gimbal_aimbot_count;
	
	uint8_t aimbot_mode;
	uint8_t aimbot_state;
	uint8_t hatch_state;
	uint8_t gimbal_mode;
	uint8_t gimbal_last_mode;
	encoding_switch_t shoot_switch;
	ins_data_t	ins_data;
	
	uint16_t shoot_count;
	uint16_t shoot_total_count;
	
	
	
	fp32 vx;
	fp32 vy;
	fp32 yaw;	//差值传入底盘
	fp32 pitch;
	
	//云台底盘差值
	fp32 err_gimbal_chassis_ecd;
	
	fp32 vx_set;
	fp32 vy_set;
	fp32 yaw_set;
	fp32 pitch_set;
	
    gimbal_motor_t gimbal_yaw_motor;	//yaw轴电机数据
	gimbal_motor_t gimbal_pitch_motor;	//pitch轴电机数据
	
	shoot_motor_t gimbal_trigger_motor;
	shoot_motor_t gimbal_fric_motor[2];
	
	//定义自瞄PID
	pid_type_def aimbot_speed_pitch_pid;
	gimbal_PID_t aimbot_angle_pitch_pid;
	pid_type_def aimbot_speed_yaw_pid;
	gimbal_PID_t aimbot_angle_yaw_pid;
	//定义陀螺仪yaw电机PID
	pid_type_def gimbal_speed_yaw_ins_pid;
	gimbal_PID_t gimbal_angle_yaw_ins_pid;
	//定义编码器yaw电机PID
	pid_type_def gimbal_speed_yaw_ecd_pid;
	gimbal_PID_t gimbal_angle_yaw_ecd_pid;
	//定义编码器pitch电机PID
	pid_type_def gimbal_speed_pitch_ecd_pid;
	gimbal_PID_t gimbal_angle_pitch_ecd_pid;
	//定义陀螺仪pitch电机PID
	pid_type_def gimbal_speed_pitch_ins_pid;
	gimbal_PID_t gimbal_angle_pitch_ins_pid;
	//定义trigger电机PID
	pid_type_def gimbal_speed_trigger_pid;
	gimbal_PID_t gimbal_angle_trigger_pid;
	//定义fric电机PID
	pid_type_def gimbal_speed_fric_pid[2];
	
} gimbal_control_t;

extern void gimbal_task(void const *pvParameters);
extern void gimbal_init(gimbal_control_t *gimbal_init);
extern void gimbal_set_mode(gimbal_control_t *gimbal_mode_set);
extern void gimbal_set_control(gimbal_control_t *gimbal_set_control_t);
extern void gimbal_feedback_update(gimbal_control_t *feedback_update);
extern void gimbal_control_loop(gimbal_control_t *gimbal_control_loop);
extern void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
extern const gimbal_control_t *get_gimbal_move_point(void);

#endif
