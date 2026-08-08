#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "can_receive.h"
#include "remote_control.h"
#include "pid.h"
#include "gimbal_task.h"


//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL			3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL			2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL			0
//在chassis_open 模型下，遥控器乘以该比例发送到can上
#define CHASSIS_OPEN_RC_SCALE		20
#define CHASSIS_OPEN_RC_SCALE_SPIN		6
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//电机减速比
#define M3508_RATIO 	 				 19	//17/268
//车轮周长 (轮子直径 * PI 再转换成mm)
#define WHEEL_PERIMETER 			 376.99111836f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 60000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//舵向电机初始化角度
#define M6020_CHASSIS_ECD_1	2014
#define	M6020_CHASSIS_ECD_2	7474
#define	M6020_CHASSIS_ECD_3	4727
#define	M6020_CHASSIS_ECD_4	7536//7861//6086

#define CHASSIS_ERR_ECD		   -PI/4.0f		//0

#define QUAR_EDC_RANGE	2048
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 	22.75555555555556f		//8192/360
#define MOTOR_RAD_TO_ECD 	0.0439453125f			//360/8192	
#define MOTOR_RAD			0.0174532925166667f		//PI/180	

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.0000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//像是小陀螺走不直，用yaw轴速度乘以这个来补偿
#define ERR_ANGLE_COMPENSATE 	0.012f

//底盘加速度比例
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘跟随速度比例，编码器模式
#define CHASSIS_FOLLOW_RATE 10.0f
//底盘跟随速度比例，陀螺仪模式
#define CHASSIS_FOLLOW_INS_RATE 5.0f
//底盘舵向角度比例
#define CHASSIS_CONTROL_STEER_FREQUENCE	1000.0f
//底盘速度比例
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL_1 0
#define CHASSIS_MODE_CHANNEL_2 1
//轮径mm
#define Radius 							0.5f
#define Spin_rate						0.5f		//小陀螺乘以这个值可以实现移动转速降低
#define PI								3.141592653589793238462643383279502884197f

//定义底盘模式
typedef enum
{
	CHASSIS_NO_MOVE,		//底盘无力
	
	CHASSIS_FOLLOW_YAW,		//底盘编码器跟随
	CHASSIS_FOLLOW_INS_YAW,	//底盘陀螺仪跟随
	
	CHASSIS_NO_FOLLOW_YAW,	//底盘单独控制
	
	CHASSIS_REGULAR_YAW,	//底盘无跟随，恒定坐标系
	CHASSIS_AIMBOT,			//用来给自瞄用
	
	CHASSIS_SPIN,			//小陀螺
	
	CHASSIS_CRUISE_NORMAL,
	CHASSIS_CRUISE_SPIN,			//自动巡航
} chassis_mode_e;

typedef enum
{
	CHASSIS_STATE_NORMAL,
	CHASSIS_MOTOR_TOE,
	CHASSIS_STEER_TOE,
	CHASSIS_ALL_TOE,
}chassis_state_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	fp32 accel;
	fp32 accel_set;
	fp32 speed;
	fp32 speed_set;
	fp32 current_set;
	int16_t give_current;
	
	fp32 absolute_angle;
	fp32 target_ecd;
} chassis_motor_t;



typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

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

} steer_motor_t;


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
} steer_pid_t;

typedef struct
{
	fp32 current;
	fp32 voltage;
	fp32 power;
	fp32 energy;
}power_control_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;
	const gimbal_control_t *chassis_gimbal_control;
	const cruise_control_t *chassis_cruise_control;
	
	power_control_t power_meter;
	
	fp32 err_angle;
	
	pid_type_def chassis_follow_yaw_pid;
	
	//3508
	chassis_motor_t chassis_motor[4];
	pid_type_def chassis_motor_pid[4];
	//6020
	steer_motor_t	chassis_steer[4];
	pid_type_def chassis_steer_speed_pid[4];
	steer_pid_t chassis_steer_angle_pid[4];
	//底盘	
	uint8_t chassis_mode;
	uint8_t chassis_last_mode;
	
	uint8_t chassis_state;
	
	fp32 vx;
	fp32 vy;
	fp32 wz;
	
	fp32 vx_set;
	fp32 vy_set;
	fp32 wz_set;
	
}chassis_move_t;


/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_set_mode(chassis_move_t *chassis_move);
extern void chassis_task(void const *pvParameters);
extern void chassis_init(chassis_move_t *chassis_move_init);
extern fp32 motor_ecd_to_angle_change(int16_t ecd, int16_t offset_ecd);
extern void chassis_set_control(chassis_move_t *chassis_move_rc_to_vector);
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
extern const chassis_move_t *get_chassis_move_point(void);
#endif
