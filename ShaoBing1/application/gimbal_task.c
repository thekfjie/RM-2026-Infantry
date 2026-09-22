#include "user_lib.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "Chassis_task.h"
#include <math.h>
#include "cmsis_os.h"
#include "arm_math.h"
#include "INS_task.h"
#include "arm_math.h"
#include "tim.h"
#include "hatch.h"
#include "gimbal_shoot_heat_control.h"
#include "bsp_laser.h"
#include "vofa.h"

#define FRIC_INSURANCE_MOUSE		1
#define FRIC_INSURANCE_RC			100
#define AIMBOT_INSURANCE	1

//aimbot pitch角度环PID
#define AIMBOT_PITCH_ANGLE_PID_KP        	28.0f	//25000.0f
#define AIMBOT_PITCH_ANGLE_PID_KI        	0.0f   //0.0f
#define AIMBOT_PITCH_ANGLE_PID_KD        	0.0f 	//200000.0f
#define AIMBOT_PITCH_ANGLE_PID_MAX_OUT   	300000.0f   //300000.0f
#define AIMBOT_PITCH_ANGLE_PID_MAX_IOUT  	50.0f   //50.	0f
                                             
//aimbot pitch速度环 PID                                          
#define AIMBOT_PITCH_SPEED_PID_KP        	16000.0f   //2.4f
#define AIMBOT_PITCH_SPEED_PID_KI        	0.0f   //0.0f
#define AIMBOT_PITCH_SPEED_PID_KD        	0.0f   //200.0f
#define AIMBOT_PITCH_SPEED_PID_MAX_OUT   	20000.0f   //10000.0f
#define AIMBOT_PITCH_SPEED_PID_MAX_IOUT  	1000.0f   //1000.0f
                                            
//aimbot yaw角度环PID                         
#define AIMBOT_YAW_ANGLE_PID_KP        				15.0f	//25000.0f
#define AIMBOT_YAW_ANGLE_PID_KI        				0.0f   //0.0f
#define AIMBOT_YAW_ANGLE_PID_KD        				0.0f 	//200000.0f
#define AIMBOT_YAW_ANGLE_PID_MAX_OUT   				300000.0f   //300000.0f
#define AIMBOT_YAW_ANGLE_PID_MAX_IOUT  				30.0f   //50.0f
                                                     
//aimbot yaw速度环 PID                                                  
#define AIMBOT_YAW_SPEED_PID_KP        				15000.0f   //2.4f
#define AIMBOT_YAW_SPEED_PID_KI        				0.0f   //0.0f
#define AIMBOT_YAW_SPEED_PID_KD        				50000.0f   //200.0f
#define AIMBOT_YAW_SPEED_PID_MAX_OUT   				30000.0f   //10000.0f
#define AIMBOT_YAW_SPEED_PID_MAX_IOUT  				1000.0f   //1000.0f
                                            
//陀螺仪yaw 角度环 PID                              
#define YAW_INS_ANGLE_PID_KP       		 		14.0f	//25000.0f
#define YAW_INS_ANGLE_PID_KI       				0.0f   //0.0f
#define YAW_INS_ANGLE_PID_KD        			0.0f 	//200000.0f
#define YAW_INS_ANGLE_PID_MAX_OUT   			300000.0f   //300000.0f
#define YAW_INS_ANGLE_PID_MAX_IOUT  			30.0f   //50.0f
                                                 
//陀螺仪yaw 速度环 PID                                             
#define YAW_INS_SPEED_PID_KP        			18000.0f   //2.4f
#define YAW_INS_SPEED_PID_KI        			0.0f   //0.0f
#define YAW_INS_SPEED_PID_KD        			50000.0f   //200.0f
#define YAW_INS_SPEED_PID_MAX_OUT   			30000.0f   //10000.0f
#define YAW_INS_SPEED_PID_MAX_IOUT  			1000.0f   //1000.0f
                                            
//编码器yaw 角度环 PID                       
#define YAW_ENCODE_ANGLE_PID_KP        		25.0f		//25.0f	
#define YAW_ENCODE_ANGLE_PID_KI        		0.0f   //0.0f
#define YAW_ENCODE_ANGLE_PID_KD        		0.0f   //0.0f
#define YAW_ENCODE_ANGLE_PID_MAX_OUT   		30000.0f   //30000.0f
#define YAW_ENCODE_ANGLE_PID_MAX_IOUT  		50.0f   //50.0f
                                             
//编码器yaw 速度环 PID                                  
#define YAW_ENCODE_SPEED_PID_KP        		0.8f   //1.0f
#define YAW_ENCODE_SPEED_PID_KI        		0.0f   //0.0f
#define YAW_ENCODE_SPEED_PID_KD        		30.0f  	//35.0f
#define YAW_ENCODE_SPEED_PID_MAX_OUT   		10000.0f   //10000.0f
#define YAW_ENCODE_SPEED_PID_MAX_IOUT  		1000.0f   //1000.0f
                                            
//编码器pitch 角度环 PID                     
#define PITCH_ENCODE_ANGLE_PID_KP        	25.0f	//25.0f
#define PITCH_ENCODE_ANGLE_PID_KI        	0.0f 	//0.0f
#define PITCH_ENCODE_ANGLE_PID_KD        	0.0f  	//0.0f
#define PITCH_ENCODE_ANGLE_PID_MAX_OUT   	30000.0f  	//30000.0f
#define PITCH_ENCODE_ANGLE_PID_MAX_IOUT  	1000.0f  	//1000.0f
                                            
//编码器pitch 速度环 PID                     
#define PITCH_ENCODE_SPEED_PID_KP        	2.1f  	//1.7f
#define PITCH_ENCODE_SPEED_PID_KI        	0.0f  	//0.0f
#define PITCH_ENCODE_SPEED_PID_KD        	0.0f  	//10.0f
#define PITCH_ENCODE_SPEED_PID_MAX_OUT   	8000.0f  	//8000.0f
#define PITCH_ENCODE_SPEED_PID_MAX_IOUT  	1000.0f  	//1000.0f

//陀螺仪pitch 角度环 PID                              
#define PITCH_INS_ANGLE_PID_KP       		 	25.0f	//25000.0f
#define PITCH_INS_ANGLE_PID_KI       			0.0f   //0.0f
#define PITCH_INS_ANGLE_PID_KD        			0.0f 	//200000.0f
#define PITCH_INS_ANGLE_PID_MAX_OUT   			300000.0f   //300000.0f
#define PITCH_INS_ANGLE_PID_MAX_IOUT  			50.0f   //50.	0f
                                                 
//陀螺仪pitch 速度环 PID                                             
#define PITCH_INS_SPEED_PID_KP        			16000.0f   //2.4f
#define PITCH_INS_SPEED_PID_KI        			0.0f   //0.0f
#define PITCH_INS_SPEED_PID_KD        			0.0f   //200.0f
#define PITCH_INS_SPEED_PID_MAX_OUT   			20000.0f   //10000.0f
#define PITCH_INS_SPEED_PID_MAX_IOUT  			1000.0f   //1000.0f

//trigger 角度环 PID
#define TRIGGER_ENCODE_ANGLE_PID_KP        20.0f
#define TRIGGER_ENCODE_ANGLE_PID_KI        0.0f
#define TRIGGER_ENCODE_ANGLE_PID_KD        0.0f
#define TRIGGER_ENCODE_ANGLE_PID_MAX_OUT   3000.0f
#define TRIGGER_ENCODE_ANGLE_PID_MAX_IOUT  50.0f
                                           
//trigger 速度环 PID
#define TRIGGER_ENCODE_SPEED_PID_KP        15.0f
#define TRIGGER_ENCODE_SPEED_PID_KI        0.0f
#define TRIGGER_ENCODE_SPEED_PID_KD        0.0f
#define TRIGGER_ENCODE_SPEED_PID_MAX_OUT   10000.0f
#define TRIGGER_ENCODE_SPEED_PID_MAX_IOUT  5000.0f

//fric 速度环 PID
#define FRIC_ENCODE_SPEED_PID_KP        15.0f
#define FRIC_ENCODE_SPEED_PID_KI        0.0f
#define FRIC_ENCODE_SPEED_PID_KD        0.0f
#define FRIC_ENCODE_SPEED_PID_MAX_OUT   10000.0f
#define FRIC_ENCODE_SPEED_PID_MAX_IOUT  1000.0f

gimbal_control_t gimbal_control;

uint32_t gimbal_high_water;


/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//云台pid计算函数
fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
//    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    pid->err = set - get;
//    pid->err = loop_fp32_constrain(err, -PI, PI);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}






void gimbal_limit(fp32 *num, fp32 max_limix, fp32 min_limit)
{
	if(*num > max_limix)
	{
		*num = max_limix;
	}
	else if(*num < min_limit)
	{
		*num = min_limit;
	}
}

//默认键鼠模式
static uint8_t control_methods = KEYBOARD_CONTROL;

//gimbal主要运行程序
void gimbal_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	
	gimbal_init(&gimbal_control);
	
	while(1)
	{
		//云台模式设定
		gimbal_set_mode(&gimbal_control);
		//云台数据获取
		gimbal_feedback_update(&gimbal_control);
		gimbal_set_control(&gimbal_control);
		gimbal_control_loop(&gimbal_control);
		
		
		//无力模式
		if(gimbal_control.gimbal_mode == CHASSIS_NO_MOVE)
		{
			CAN_cmd_gimbal(0, 0);
			CAN_cmd_shoot(0, 0, 0);
			hatch_no_move();
		}
		//底盘跟随云台模式和纯底盘运动和小陀螺模式，给云台赋予电流
		else
		{
			CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.give_current,gimbal_control.gimbal_pitch_motor.give_current);
			CAN_cmd_shoot(gimbal_control.gimbal_trigger_motor.give_current, gimbal_control.gimbal_fric_motor[0].give_current, gimbal_control.gimbal_fric_motor[1].give_current);
			
		}

		
		vTaskDelay(GIMBAL_CONTROL_TIME);
		
#if INCLUDE_uxTaskGetStackHighWaterMark
		gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}


void gimbal_init(gimbal_control_t *gimbal_task_init)
{
	if (gimbal_task_init == NULL)
    {
        return;
    }
	
	shoot_init();
	//起始值赋0
	gimbal_task_init->yaw = 0;
	gimbal_task_init->pitch = 0;
	gimbal_task_init->shoot_count = 0;
	gimbal_task_init->gimbal_pitch_motor.relative_angle_set = M6020_GIMBAL_ANGLE_PITCH;
	
	if(control_methods == KEYBOARD_CONTROL)
		gimbal_task_init->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
	else
		gimbal_task_init->gimbal_mode = CHASSIS_NO_MOVE;	//云台、底盘初始化无力
	gimbal_task_init->shoot_switch.fric_flag = FRIC_READY;
	gimbal_task_init->shoot_switch.fric_mode = FRIC_STOP;	//发射机构初始化无力
	gimbal_task_init->shoot_switch.shoot_mode = SHOOT_STOP;
	gimbal_task_init->shoot_switch.shoot_state = SHOOT_SINGLE_START_STATE;
	gimbal_task_init->hatch_state = HATCH_CLOSE;
	
	//陀螺仪数据指针获取
	gimbal_task_init->gimbal_INT_angle_point = get_INS_angle_point();
	gimbal_task_init->gimbal_INT_gyro_point = get_gyro_data_point();
	//获取自瞄数据指针
	gimbal_task_init->gimbal_aimbot_point = get_aimbot_point();
	//遥控器数据指针获取
	gimbal_task_init->gimbal_rc_ctrl = get_remote_control_point();
	//获得巡航数据
	gimbal_task_init->gimbal_cruise_point = get_chassis_cruise_point();
	//电机数据指针获取
	gimbal_task_init->gimbal_yaw_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(0);
	gimbal_task_init->gimbal_pitch_motor.gimbal_motor_measure = get_gimbal_motor_measure_point(1);
	gimbal_task_init->gimbal_trigger_motor.shoot_motor_measure = get_shoot_motor_measure_point(0);
	gimbal_task_init->gimbal_fric_motor[0].shoot_motor_measure = get_shoot_motor_measure_point(1);
	gimbal_task_init->gimbal_fric_motor[1].shoot_motor_measure = get_shoot_motor_measure_point(2);
	
	//初始化云台电机PID参数
	const static fp32 aimbot_speed_pitch_pid[3] = {AIMBOT_PITCH_SPEED_PID_KP, AIMBOT_PITCH_SPEED_PID_KI, AIMBOT_PITCH_SPEED_PID_KD};
	const static fp32 aimbot_speed_yaw_pid[3] = {AIMBOT_YAW_SPEED_PID_KP, AIMBOT_YAW_SPEED_PID_KI, AIMBOT_YAW_SPEED_PID_KD};
	
	const static fp32 gimbal_speed_yaw_ins_pid[3] = {YAW_INS_SPEED_PID_KP, YAW_INS_SPEED_PID_KI, YAW_INS_SPEED_PID_KD};
	const static fp32 gimbal_speed_yaw_ecd_pid[3] = {YAW_ENCODE_SPEED_PID_KP, YAW_ENCODE_SPEED_PID_KI, YAW_ENCODE_SPEED_PID_KD};
	const static fp32 gimbal_speed_pitch_ecd_pid[3] = {PITCH_ENCODE_SPEED_PID_KP, PITCH_ENCODE_SPEED_PID_KI, PITCH_ENCODE_SPEED_PID_KD};
	const static fp32 gimbal_speed_pitch_ins_pid[3] = {PITCH_INS_SPEED_PID_KP, PITCH_INS_SPEED_PID_KI, PITCH_INS_SPEED_PID_KD};
	const static fp32 gimbal_speed_trigger_pid[3] = {TRIGGER_ENCODE_SPEED_PID_KP, TRIGGER_ENCODE_SPEED_PID_KI, TRIGGER_ENCODE_SPEED_PID_KD};
	const static fp32 gimbal_speed_fric_pid[3] = {FRIC_ENCODE_SPEED_PID_KP, FRIC_ENCODE_SPEED_PID_KI, FRIC_ENCODE_SPEED_PID_KD};
	//初始化自瞄PID
	gimbal_PID_init(&gimbal_task_init->aimbot_angle_pitch_pid, AIMBOT_PITCH_ANGLE_PID_MAX_OUT, AIMBOT_PITCH_ANGLE_PID_MAX_IOUT, AIMBOT_PITCH_ANGLE_PID_KP, AIMBOT_PITCH_ANGLE_PID_KI, AIMBOT_PITCH_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->aimbot_speed_pitch_pid, PID_POSITION, aimbot_speed_pitch_pid, AIMBOT_PITCH_SPEED_PID_MAX_OUT, AIMBOT_PITCH_SPEED_PID_MAX_IOUT);
	gimbal_PID_init(&gimbal_task_init->aimbot_angle_yaw_pid, AIMBOT_YAW_ANGLE_PID_MAX_OUT, AIMBOT_YAW_ANGLE_PID_MAX_IOUT, AIMBOT_YAW_ANGLE_PID_KP, AIMBOT_YAW_ANGLE_PID_KI, AIMBOT_YAW_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->aimbot_speed_yaw_pid, PID_POSITION, aimbot_speed_yaw_pid, AIMBOT_YAW_SPEED_PID_MAX_OUT, AIMBOT_YAW_SPEED_PID_MAX_IOUT);
	//初始化陀螺仪yaw电机PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_yaw_ins_pid, YAW_INS_ANGLE_PID_MAX_OUT, YAW_INS_ANGLE_PID_MAX_IOUT, YAW_INS_ANGLE_PID_KP, YAW_INS_ANGLE_PID_KI, YAW_INS_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_yaw_ins_pid, PID_POSITION, gimbal_speed_yaw_ins_pid, YAW_INS_SPEED_PID_MAX_OUT, YAW_INS_SPEED_PID_MAX_IOUT);
	//初始化编码器yaw电机PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_yaw_ecd_pid, YAW_ENCODE_ANGLE_PID_MAX_OUT, YAW_ENCODE_ANGLE_PID_MAX_IOUT, YAW_ENCODE_ANGLE_PID_KP, YAW_ENCODE_ANGLE_PID_KI, YAW_ENCODE_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_yaw_ecd_pid, PID_POSITION, gimbal_speed_yaw_ecd_pid, YAW_ENCODE_SPEED_PID_MAX_OUT, YAW_ENCODE_SPEED_PID_MAX_IOUT);
	//初始化编码器pitch电机PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_pitch_ecd_pid, PITCH_ENCODE_ANGLE_PID_MAX_OUT, PITCH_ENCODE_ANGLE_PID_MAX_IOUT, PITCH_ENCODE_ANGLE_PID_KP, PITCH_ENCODE_ANGLE_PID_KI, PITCH_ENCODE_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_pitch_ecd_pid, PID_POSITION, gimbal_speed_pitch_ecd_pid, PITCH_ENCODE_SPEED_PID_MAX_OUT, PITCH_ENCODE_SPEED_PID_MAX_IOUT);
	//初始化陀螺仪pitch电机PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_pitch_ins_pid, PITCH_INS_ANGLE_PID_MAX_OUT, PITCH_INS_ANGLE_PID_MAX_IOUT, PITCH_INS_ANGLE_PID_KP, PITCH_INS_ANGLE_PID_KI, PITCH_INS_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_pitch_ins_pid, PID_POSITION, gimbal_speed_pitch_ins_pid, PITCH_INS_SPEED_PID_MAX_OUT, PITCH_INS_SPEED_PID_MAX_IOUT);
	//初始化trigger电机PID
	gimbal_PID_init(&gimbal_task_init->gimbal_angle_trigger_pid, TRIGGER_ENCODE_ANGLE_PID_MAX_OUT, TRIGGER_ENCODE_ANGLE_PID_MAX_IOUT, TRIGGER_ENCODE_ANGLE_PID_KP, TRIGGER_ENCODE_ANGLE_PID_KI, TRIGGER_ENCODE_ANGLE_PID_KD);
	PID_init(&gimbal_task_init->gimbal_speed_trigger_pid, PID_POSITION, gimbal_speed_trigger_pid, TRIGGER_ENCODE_SPEED_PID_MAX_OUT, TRIGGER_ENCODE_SPEED_PID_MAX_IOUT);
	//初始化摩擦轮电机PID
	for(int i=0; i<2; i++)
		PID_init(&gimbal_task_init->gimbal_speed_fric_pid[i], PID_POSITION, gimbal_speed_fric_pid, FRIC_ENCODE_SPEED_PID_MAX_OUT, FRIC_ENCODE_SPEED_PID_MAX_IOUT);
	//初始化舱门PWM
	hatch_init();
	//初始化陀螺仪模式起始值（需要加一个PI，因为起始值默认有一个-PI）
	gimbal_task_init->ins_data.yaw_target = gimbal_task_init->ins_data.yaw+PI;	//除了小陀螺模式下陀螺仪target赋值
	gimbal_task_init->ins_data.pitch_target = gimbal_task_init->ins_data.pitch;
	//初始化激光
	laser_init();
}

void gimbal_set_mode(gimbal_control_t *gimbal_mode_set)
{
	if (gimbal_mode_set == NULL)
    {
        return;
    }
	//底盘模式切换
	if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]))
	{
		//底盘无力
		gimbal_mode_set->gimbal_mode = CHASSIS_NO_MOVE;
	}
	if(control_methods == REMOTE_CONTROL)
	{
		if(switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//自动巡航模式
			gimbal_mode_set->gimbal_mode = CHASSIS_CRUISE_NORMAL;
		}
		else if(switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//自动巡航模式,小陀螺
			gimbal_mode_set->gimbal_mode = CHASSIS_CRUISE_SPIN;
		}
		else if(switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//云台跟随底盘，编码器控制
			gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_YAW;
		}
		else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//纯底盘控制
			gimbal_mode_set->gimbal_mode = CHASSIS_NO_FOLLOW_YAW;
		}
		else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//底盘固定方向控制
			gimbal_mode_set->gimbal_mode = CHASSIS_REGULAR_YAW;
		}
		else if(switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//纯云台，编码器控制
//			自瞄
			gimbal_mode_set->gimbal_mode = CHASSIS_AIMBOT;
		}
		else if(switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//小陀螺
			gimbal_mode_set->gimbal_mode = CHASSIS_SPIN;
		}
		else if(switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]))
		{
			//云台跟随地盘，陀螺仪控制
			gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
		}
	}
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].b == 1)
		control_methods = REMOTE_CONTROL;
//	else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].v == 1)
//		control_methods = REMOTE_CONTROL;
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].g == 1)
		HAL_NVIC_SystemReset();										//重启C板
	//用键盘控制的几个主要的模式
	//底盘无力和正常模式切换
	if(control_methods == KEYBOARD_CONTROL)
	{
//		if(gimbal_mode_set->gimbal_mode == CHASSIS_NO_MOVE)
//		{
//			gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;	//暂时
//		}
		
		
		if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].z == 1)
			gimbal_mode_set->gimbal_mode = CHASSIS_NO_MOVE;				//底盘无力	
		else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].z == 1 && gimbal_mode_set->gimbal_mode == CHASSIS_NO_MOVE)
			gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
		
		if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].x == 1)
			gimbal_mode_set->gimbal_mode = CHASSIS_REGULAR_YAW;			//底盘固定
//		else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].c == 1)
//			gimbal_mode_set->gimbal_mode = CHASSIS_SPIN;			//底盘固定
		
		static uint8_t mode_change_state = 0;
		//陀螺仪yaw轴正常控制和小陀螺q键切换
		if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].q == 1 && gimbal_mode_set->gimbal_mode == CHASSIS_SPIN&&mode_change_state == 0)
		{
			gimbal_mode_set->gimbal_mode = CHASSIS_FOLLOW_INS_YAW;
			mode_change_state = 1;
		}
		else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].q == 1 && gimbal_mode_set->gimbal_mode == CHASSIS_FOLLOW_INS_YAW&&mode_change_state == 0)
		{
			gimbal_mode_set->gimbal_mode = CHASSIS_SPIN;
			mode_change_state = 1;
		}
		else if(mode_change_state == 1&&gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].q == 0)
			mode_change_state = 0;
		

	}
	//发射机构模式切换
	//获得拨弹开关编码器值
	gimbal_mode_set->shoot_switch.encoding_switch_ecd = gimbal_mode_set->gimbal_rc_ctrl->rc.ch[SHOOT_MODE_CHANNEL];
	
	if(gimbal_mode_set->gimbal_mode == CHASSIS_NO_MOVE)
	{
		//摩擦轮停止
		gimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
		//拨弹轮停止
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_STOP;
	}

	static uint16_t shoot_i;
	if((gimbal_mode_set->shoot_switch.fric_mode == FRIC_STOP) && (gimbal_mode_set->shoot_switch.fric_flag == FRIC_READY) && (gimbal_mode_set->shoot_switch.encoding_switch_ecd<=-400))
	{
		shoot_i++;
		if(shoot_i >= FRIC_INSURANCE_RC)
		{
			gimbal_mode_set->shoot_switch.fric_mode = FRIC_START;
			gimbal_mode_set->shoot_switch.fric_flag = FRIC_START;
			shoot_i=0;
		}
	}
	else if((gimbal_mode_set->shoot_switch.fric_mode == FRIC_START) && (gimbal_mode_set->shoot_switch.fric_flag == FRIC_READY) && (gimbal_mode_set->shoot_switch.encoding_switch_ecd<=-400))
	{
		gimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
	}
	else if(ABS(gimbal_mode_set->shoot_switch.encoding_switch_ecd)<10)
	{
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_READY;
	}
	//鼠标开启关闭摩擦轮
	if(gimbal_mode_set->shoot_switch.fric_mode == FRIC_STOP && (gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 1))
	{
		shoot_i++;
		if(shoot_i>= FRIC_INSURANCE_MOUSE)
		{
			gimbal_mode_set->shoot_switch.fric_mode = FRIC_START;
			gimbal_mode_set->shoot_switch.fric_flag = FRIC_START;
		}
	}
	else if(gimbal_mode_set->shoot_switch.fric_mode == FRIC_START && gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].e == 1)
	{
		gimbal_mode_set->shoot_switch.fric_mode = FRIC_STOP;
		gimbal_mode_set->shoot_switch.fric_flag = FRIC_STOP;
		shoot_i=0;
	}
	
	//自瞄控制
	if((gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r == 1)&&(gimbal_mode_set->aimbot_state == AIMBOT_GET))				//鼠标右键
		gimbal_mode_set->aimbot_mode = AIMBOT_ON;
	else
		gimbal_mode_set->aimbot_mode = AIMBOT_OFF;
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].r == 1)
		gimbal_mode_set->aimbot_mode = AIMBOT_RESET;
	//手柄自瞄
	if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_1]) &&                    //左右拨杆最上
    switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL_2]) &&
    gimbal_mode_set->aimbot_state == AIMBOT_GET) 
	{
    gimbal_mode_set->aimbot_mode = AIMBOT_ON;
	}
	else
		gimbal_mode_set->aimbot_mode = AIMBOT_OFF;
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].r == 1)
		gimbal_mode_set->aimbot_mode = AIMBOT_RESET;
	
	//拨弹轮控制
	//f键爆发控制
	static uint16_t shoot_state_flag;
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].f == 1&&shoot_state_flag == 0)
	{
		if(gimbal_mode_set->shoot_switch.shoot_state==SHOOT_SINGLE_START_STATE)
			gimbal_mode_set->shoot_switch.shoot_state = SHOOT_CONTINUOUS_START_STATE;
		else
			gimbal_mode_set->shoot_switch.shoot_state = SHOOT_SINGLE_START_STATE;
		shoot_state_flag = 1;
	}
	else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].f == 0)
		shoot_state_flag = 0;
	//v键解除控制
	static uint8_t shoot_heat_limit_flag;
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].v == 1&&shoot_heat_limit_flag == 0)
	{
		if(gimbal_mode_set->shoot_switch.shoot_state!=SHOOT_WITHOUT_CONTROL_STATE)
			gimbal_mode_set->shoot_switch.shoot_state = SHOOT_WITHOUT_CONTROL_STATE;
		else if(gimbal_mode_set->shoot_switch.shoot_state==SHOOT_WITHOUT_CONTROL_STATE)
			gimbal_mode_set->shoot_switch.shoot_state = SHOOT_CONTINUOUS_START_STATE;
		shoot_heat_limit_flag = 1;
	}
	else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].v == 0)
		shoot_heat_limit_flag = 0;

	if(gimbal_mode_set->shoot_switch.encoding_switch_ecd>200&&gimbal_mode_set->shoot_switch.encoding_switch_ecd<450)	//遥控器编码轮
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_SINGLE_START;
	else if(gimbal_mode_set->shoot_switch.encoding_switch_ecd>450)
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;
	else if(gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 1&&fabs(gimbal_mode_set->gimbal_fric_motor[0].speed)>10)				//鼠标左键
		switch(gimbal_mode_set->shoot_switch.shoot_state)
		{
			case SHOOT_CONTINUOUS_START_STATE:gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_CONTINUOUS_START;
				break;
			case SHOOT_SINGLE_START_STATE:gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_SINGLE_START;
				break;
			case SHOOT_WITHOUT_CONTROL_STATE:gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_WITHOUT_CONTROL;
				break;
		}
	//自瞄自动开火
//	else if(gimbal_mode_set->aimbot_mode == AIMBOT_ON&&fabs(gimbal_mode_set->yaw_set-gimbal_mode_set->yaw)<AIMBOT_AUTO_FIRE_THRE)
//		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_SINGLE_START;
	else
		gimbal_mode_set->shoot_switch.shoot_mode = SHOOT_STOP;
	
	//舱门控制键盘
	static uint8_t hatch_state_flag;
	if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].c == 1&&hatch_state_flag == 0)
	{
		if(gimbal_mode_set->hatch_state==HATCH_CLOSE)
			gimbal_mode_set->hatch_state = HATCH_OPEN;
		else if(gimbal_mode_set->hatch_state==HATCH_OPEN)
			gimbal_mode_set->hatch_state = HATCH_CLOSE;
		hatch_state_flag = 1;
	}
	else if(gimbal_mode_set->gimbal_rc_ctrl->key[KEY_PRESS].c == 0)
		hatch_state_flag = 0;
	//遥控
	if((gimbal_mode_set->shoot_switch.encoding_switch_ecd>400&&gimbal_mode_set->shoot_switch.fric_mode == FRIC_STOP)||gimbal_mode_set->hatch_state==HATCH_OPEN)
	{
		//打开舱门
		hatch_open();
	}
	else
	{
		hatch_close();
	}
}

//陀螺仪过零点计算
void gimbal_ins_data_count(ins_data_t *ins_data_count)
{
	
	ins_data_count->yaw += PI;
	while(ins_data_count->yaw_last-ins_data_count->yaw>PI)
		ins_data_count->yaw += PI*2;
	while(ins_data_count->yaw_last-ins_data_count->yaw<-PI)
		ins_data_count->yaw -= PI*2;
	ins_data_count->yaw_last = ins_data_count->yaw;
}

void gimbal_feedback_update(gimbal_control_t *gimbal_task_update)
{
	if (gimbal_task_update == NULL)
    {
        return;
    }
	//获取陀螺仪数据
	gimbal_task_update->ins_data.roll = gimbal_task_update->gimbal_INT_angle_point[ROLL_CHANNEL];
	gimbal_task_update->ins_data.pitch = gimbal_task_update->gimbal_INT_angle_point[PITCH_CHANNEL];
	gimbal_task_update->ins_data.yaw = gimbal_task_update->gimbal_INT_angle_point[YAW_CHANNEL] - gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER;
	gimbal_ins_data_count(&gimbal_task_update->ins_data);
	//获得yaw轴6020数据
	gimbal_task_update->gimbal_yaw_motor.speed = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
    gimbal_task_update->gimbal_yaw_motor.absolute_angle = gimbal_task_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
	//获得pitch轴6020数据
	gimbal_task_update->gimbal_pitch_motor.speed = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
    gimbal_task_update->gimbal_pitch_motor.absolute_angle = gimbal_task_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
	//获得发射机构电机数据
	gimbal_task_update->gimbal_trigger_motor.speed = gimbal_task_update->gimbal_trigger_motor.shoot_motor_measure->speed_rpm;
	gimbal_task_update->gimbal_trigger_motor.relative_angle = gimbal_task_update->gimbal_trigger_motor.shoot_motor_measure->ecd;

	//获得摩擦轮电机数据
	for(int i=0; i<2; i++)
	{
//		//测试拨弹
//		uint16_t speed_rpm=1000;
//		uint16_t speed_ecd=1000;
		gimbal_task_update->gimbal_fric_motor[i].speed = gimbal_task_update->gimbal_fric_motor[i].shoot_motor_measure->speed_rpm;
		gimbal_task_update->gimbal_fric_motor[i].relative_angle = gimbal_task_update->gimbal_fric_motor[i].shoot_motor_measure->ecd;
	}
	
	aimbot_mode_set(gimbal_task_update->aimbot_mode);	//传出自瞄模式数据
	gimbal_task_update->aimbot_state = gimbal_task_update->gimbal_aimbot_point->aimbot_state;
	
	gimbal_task_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_task_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(gimbal_task_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
	gimbal_task_update->gimbal_pitch_motor.motor_gyro = *(gimbal_task_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
}


void gimbal_set_control(gimbal_control_t *gimbal_set_control_t)
{
	if (gimbal_set_control_t == NULL)
    {
        return;
    }
	
	//遥控器对yaw,pitch控制赋值
	if(gimbal_set_control_t->gimbal_mode != CHASSIS_NO_MOVE)
	{
		if(gimbal_set_control_t->aimbot_mode == AIMBOT_ON)
		{
			gimbal_set_control_t->yaw_set = gimbal_set_control_t->gimbal_aimbot_point->target_yaw;
			gimbal_set_control_t->pitch_set = gimbal_set_control_t->gimbal_aimbot_point->target_pitch;
		}
		else if(gimbal_set_control_t->gimbal_mode == CHASSIS_CRUISE_NORMAL||gimbal_set_control_t->gimbal_mode == CHASSIS_CRUISE_SPIN)
		{
			gimbal_set_control_t->yaw_set = gimbal_set_control_t->gimbal_cruise_point->yaw;
			gimbal_set_control_t->pitch_set = gimbal_set_control_t->gimbal_cruise_point->pitch;
		}
		else
		{
			gimbal_set_control_t->yaw_set = -(gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_YAW_CHANNEL] * GIMBAL_OPEN_RC_YAW_SEN+gimbal_set_control_t->gimbal_rc_ctrl->mouse.x*GIMBAL_OPEN_X_YAW_SEN);
			if(gimbal_set_control_t->gimbal_mode == CHASSIS_FOLLOW_YAW||
				gimbal_set_control_t->gimbal_mode == CHASSIS_NO_FOLLOW_YAW)
				gimbal_set_control_t->pitch_set = (gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_PITCH_CHANNEL] * GIMBAO_OPEN_RC_PITCH_FAST_SEN-gimbal_set_control_t->gimbal_rc_ctrl->mouse.y*GIMBAL_OPEN_Y_PITCH_SEN);
			else
				gimbal_set_control_t->pitch_set = (gimbal_set_control_t->gimbal_rc_ctrl->rc.ch[RC_PITCH_CHANNEL] * GIMBAO_OPEN_RC_PITCH_FAST_SEN-gimbal_set_control_t->gimbal_rc_ctrl->mouse.y*GIMBAL_OPEN_Y_PITCH_SEN);
		}
	}
	
	if(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START)
	{
		gimbal_control.gimbal_fric_motor[0].speed_set = FRIC_SHOOT_LEFT_SPEED;
		gimbal_control.gimbal_fric_motor[1].speed_set = -FRIC_SHOOT_RIGHT_SPEED;
	}
	else if(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_STOP)
	{
		gimbal_control.gimbal_fric_motor[0].speed_set = 0;
		gimbal_control.gimbal_fric_motor[1].speed_set = 0;
		gimbal_control.gimbal_trigger_motor.speed_set = 0;
	}
	
	//遥控器对拨弹轮trigger控制赋值
	//trigger电机逻辑部分
	if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_SINGLE_START)&&(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_SLOW;
	else if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_CONTINUOUS_START)&&(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_FAST;
	else if((gimbal_set_control_t->shoot_switch.shoot_mode == SHOOT_WITHOUT_CONTROL)&&(gimbal_set_control_t->shoot_switch.fric_mode == FRIC_START))
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_SPEED_RATE_FAST;
	else
	{	
		gimbal_set_control_t->gimbal_trigger_motor.shoot_delay = TRIGGER_SHOOT_STOP;
	}
	shoot_task(&gimbal_set_control_t->shoot_switch.trigger_state , gimbal_set_control_t->shoot_switch.shoot_mode, gimbal_set_control_t->shoot_switch.fric_mode, &gimbal_set_control_t->gimbal_trigger_motor.speed_set, gimbal_set_control_t->gimbal_trigger_motor.shoot_delay);
	
}


//底盘舵向起转判断
void gimbal_to_chassis_follow_yaw_determine(gimbal_control_t *gimbal_chassis_follow_yaw)
{	
	//底盘跟随模式计算编码器差值传入底盘
	if(gimbal_chassis_follow_yaw->gimbal_yaw_motor.absolute_angle - M6020_GIMBAL_ANGLE_YAW != 0)
		gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd = gimbal_chassis_follow_yaw->gimbal_yaw_motor.absolute_angle - M6020_GIMBAL_ANGLE_YAW;
	//优劣弧计算
	if(gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd>HALF_ECD_RANGE)			
		gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd -= ECD_RANGE;
	else if(gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd<-HALF_ECD_RANGE)		
		gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd += ECD_RANGE;
	//过幅限制
//	abs_limit(&gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd, GIMBAO_TURN_LIMIT);
	
	
		gimbal_chassis_follow_yaw->yaw = gimbal_chassis_follow_yaw->err_gimbal_chassis_ecd;
}





//编码器云台yaw
void gimbal_ecd_yaw_control(gimbal_control_t *gimbal_ecd)
{
	//云台yaw轴控制和回位
	gimbal_ecd->gimbal_yaw_motor.absolute_angle_set = gimbal_ecd->yaw_set + M6020_GIMBAL_ANGLE_YAW;
	//云台目标值赋值
	gimbal_ecd->gimbal_yaw_motor.target_ecd = motor_ecd_to_angle_change(gimbal_ecd->gimbal_yaw_motor.absolute_angle_set, 
																		gimbal_ecd->gimbal_yaw_motor.absolute_angle);
	//yaw轴pid运算
	gimbal_ecd->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_ecd->gimbal_angle_yaw_ecd_pid, gimbal_ecd->gimbal_yaw_motor.absolute_angle, 
																gimbal_ecd->gimbal_yaw_motor.target_ecd, gimbal_ecd->gimbal_yaw_motor.speed);
	gimbal_ecd->gimbal_yaw_motor.current_set = PID_calc(&gimbal_ecd->gimbal_speed_yaw_ecd_pid, gimbal_ecd->gimbal_yaw_motor.speed, 
														gimbal_ecd->gimbal_yaw_motor.motor_gyro_set);
}

void aimbot_ecd_pitch_control(gimbal_control_t *gimbal_ecd_pitch)
{
	
}

//陀螺仪云台yaw
void gimbal_ins_yaw_control(gimbal_control_t *gimbal_ins)
{
	if(gimbal_ins->aimbot_mode == AIMBOT_OFF)
	{
		gimbal_ins->gimbal_yaw_motor.add += gimbal_ins->yaw_set*YAW_INS_OPEN_RC_YAW_SEN;
		gimbal_ins->gimbal_yaw_motor.absolute_angle_set = gimbal_ins->ins_data.yaw_target + gimbal_ins->gimbal_yaw_motor.add;
		
		//计算陀螺仪差值
		gimbal_ins->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&gimbal_ins->gimbal_angle_yaw_ins_pid , gimbal_ins->ins_data.yaw, 
																				gimbal_ins->gimbal_yaw_motor.absolute_angle_set, gimbal_ins->gimbal_yaw_motor.motor_gyro);
		gimbal_ins->gimbal_yaw_motor.current_set = PID_calc(&gimbal_ins->gimbal_speed_yaw_ins_pid, gimbal_ins->gimbal_yaw_motor.motor_gyro, 
																		gimbal_ins->gimbal_yaw_motor.motor_gyro_set);
	}
}

//自瞄云台，已经在set_control加入遥控器输入值，就是结合了陀螺仪云台
static uint16_t gimbal_aimbot_delay;
void aimbot_gimbal_control(gimbal_control_t *aimbot_gimbal_control_point)
{
	fp32 yaw_error;
	if(aimbot_gimbal_control_point->aimbot_mode == AIMBOT_ON)
	{
		gimbal_aimbot_delay++;
		if(gimbal_aimbot_delay >= AIMBOT_INSURANCE)
		{
			//pitch
			if(fabs(aimbot_gimbal_control_point->gimbal_aimbot_point->target_pitch-aimbot_gimbal_control_point->gimbal_INT_angle_point[PITCH_CHANNEL])<PI/4.0f)
				aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_aimbot_point->target_pitch;
			else
				aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_INT_angle_point[PITCH_CHANNEL];
			//PID——pitch
			aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_pitch_pid , aimbot_gimbal_control_point->ins_data.pitch, 
																					aimbot_gimbal_control_point->gimbal_pitch_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro);
			aimbot_gimbal_control_point->gimbal_pitch_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_pitch_pid, aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro, 
																			aimbot_gimbal_control_point->gimbal_pitch_motor.motor_gyro_set);
			//yaw
			yaw_error = loop_fp32_constrain(
				aimbot_gimbal_control_point->gimbal_aimbot_point->target_yaw -
				aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL], -PI, PI);
			if(fabs(yaw_error)<PI/8.0f)
				aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set =
					aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL] + yaw_error;
			else
				aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set = aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL];
				
			//PID——yaw
			aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set = gimbal_PID_calc(&aimbot_gimbal_control_point->aimbot_angle_yaw_pid , aimbot_gimbal_control_point->gimbal_INT_angle_point[YAW_CHANNEL] - aimbot_gimbal_control_point->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*GIMBAL_SPIN_YAW_BUFFER, 
																						aimbot_gimbal_control_point->gimbal_yaw_motor.absolute_angle_set, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro);
			aimbot_gimbal_control_point->gimbal_yaw_motor.current_set = PID_calc(&aimbot_gimbal_control_point->aimbot_speed_yaw_pid, aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro, 
																				aimbot_gimbal_control_point->gimbal_yaw_motor.motor_gyro_set);
		}
	}
	else if(aimbot_gimbal_control_point->aimbot_mode == AIMBOT_OFF)
		gimbal_aimbot_delay = 0;
}



//小陀螺模式传出编码器差值
void gimbal_to_chassis_spin(gimbal_control_t *gimbal_chassis_spin)
{
	gimbal_chassis_spin->err_gimbal_chassis_ecd = gimbal_chassis_spin->gimbal_yaw_motor.absolute_angle - M6020_GIMBAL_ANGLE_YAW;
	gimbal_chassis_spin->yaw = gimbal_chassis_spin->err_gimbal_chassis_ecd;
}

//固定底盘模式
void gimbal_to_chassis_regular(gimbal_control_t *gimbal_chassis_regular)
{
	//不传云台编码器差值，实现底盘固定
}




void fric_motor_PID_control(gimbal_control_t *fric_motore_control)
{
	for(int i=0; i<2; i++)
	{
		fric_motore_control->gimbal_fric_motor[i].current_set = PID_calc(&fric_motore_control->gimbal_speed_fric_pid[i], fric_motore_control->gimbal_fric_motor[i].speed, fric_motore_control->gimbal_fric_motor[i].speed_set);
		//shoot电机电流赋值
		fric_motore_control->gimbal_fric_motor[i].give_current = fric_motore_control->gimbal_fric_motor[i].current_set;
	}
}

//云台发射机构控制
//static uint8_t circle = 0;
//static uint16_t count = 0;
//static uint32_t absolut_angle = 0;
void trigger_motor_PID_control(gimbal_control_t *trigger_motore_control)
{	

//	gimbal_shoot_heat_control(trigger_motore_control);
	trigger_motore_control->gimbal_trigger_motor.target_ecd += trigger_motore_control->gimbal_trigger_motor.speed_set;
//	trigger_motore_control->gimbal_trigger_motor.speed_set = -(trigger_motore_control->gimbal_trigger_motor.speed_set);

	//trigger电机PID运算
//	trigger_motore_control->gimbal_trigger_motor.motor_gyro_set = gimbal_PID_calc(&trigger_motore_control->gimbal_angle_trigger_pid, trigger_motore_control->gimbal_trigger_motor.absolute_angle*count, trigger_motore_control->gimbal_trigger_motor.target_ecd, trigger_motore_control->gimbal_trigger_motor.speed);
	trigger_motore_control->gimbal_trigger_motor.current_set = PID_calc(&trigger_motore_control->gimbal_speed_trigger_pid, trigger_motore_control->gimbal_trigger_motor.speed, trigger_motore_control->gimbal_trigger_motor.speed_set);
	//trigger电机电流赋值
	trigger_motore_control->gimbal_trigger_motor.give_current = trigger_motore_control->gimbal_trigger_motor.current_set;
	
	
	
}

void pitch_ecd_PID_control(gimbal_control_t *pitch_ecd_control)
{
	//云台pitch轴控制和回位
	pitch_ecd_control->gimbal_pitch_motor.relative_angle_set += pitch_ecd_control->pitch_set*GIMBAL_PITCH_ECD_RATE;
	//pitch轴编码器限幅
	gimbal_limit(&pitch_ecd_control->gimbal_pitch_motor.relative_angle_set, PITCH_ECD_MAX, PITCH_ECD_MIN);
	pitch_ecd_control->gimbal_pitch_motor.target_ecd = motor_ecd_to_angle_change(pitch_ecd_control->gimbal_pitch_motor.relative_angle_set, pitch_ecd_control->gimbal_pitch_motor.absolute_angle);
	//pitch轴PID运算
	pitch_ecd_control->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_ecd_control->gimbal_angle_pitch_ecd_pid, pitch_ecd_control->gimbal_pitch_motor.absolute_angle, pitch_ecd_control->gimbal_pitch_motor.target_ecd, pitch_ecd_control->gimbal_pitch_motor.speed);
	pitch_ecd_control->gimbal_pitch_motor.current_set = PID_calc(&pitch_ecd_control->gimbal_speed_pitch_ecd_pid, pitch_ecd_control->gimbal_pitch_motor.speed, pitch_ecd_control->gimbal_pitch_motor.motor_gyro_set);
}

void pitch_ins_PID_control(gimbal_control_t *pitch_ins_control)
{
	if(pitch_ins_control->aimbot_mode == AIMBOT_OFF)
	{
		if(pitch_ins_control->gimbal_pitch_motor.absolute_angle_set < PITCH_INS_MAX || pitch_ins_control->gimbal_pitch_motor.absolute_angle_set>-PITCH_INS_MIN)
			pitch_ins_control->gimbal_pitch_motor.add += pitch_ins_control->pitch_set*GIMBAL_PITCH_INS_RATE;
		pitch_ins_control->gimbal_pitch_motor.absolute_angle_set = pitch_ins_control->gimbal_pitch_motor.add + pitch_ins_control->ins_data.pitch_target;
		//pitch轴
		gimbal_limit(&pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, PITCH_INS_MAX, -PITCH_INS_MIN);
		
		pitch_ins_control->gimbal_pitch_motor.motor_gyro_set = gimbal_PID_calc(&pitch_ins_control->gimbal_angle_pitch_ins_pid , pitch_ins_control->ins_data.pitch, 
																				pitch_ins_control->gimbal_pitch_motor.absolute_angle_set, pitch_ins_control->gimbal_pitch_motor.motor_gyro);
		pitch_ins_control->gimbal_pitch_motor.current_set = PID_calc(&pitch_ins_control->gimbal_speed_pitch_ins_pid, pitch_ins_control->gimbal_pitch_motor.motor_gyro, 
																		pitch_ins_control->gimbal_pitch_motor.motor_gyro_set);
	}
}

void gimbal_control_loop(gimbal_control_t *gimbal_control_loop_t)
{

	//底盘舵向起转判断
	if(gimbal_control_loop_t->gimbal_mode == CHASSIS_FOLLOW_YAW)
	{
		gimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);	//底盘跟随云台模式
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_FOLLOW_INS_YAW)
	{
		gimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);	//底盘跟随云台模式
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_CRUISE_NORMAL)
	{
		gimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);	//底盘跟随云台模式
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_SPIN)
	{
		gimbal_to_chassis_spin(gimbal_control_loop_t);	//小陀螺
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_CRUISE_SPIN)
	{
		gimbal_to_chassis_spin(gimbal_control_loop_t);	//小陀螺
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_REGULAR_YAW)
	{
		gimbal_to_chassis_follow_yaw_determine(gimbal_control_loop_t);
	}
	
	
	//这几个模式要用到陀螺仪
	if((gimbal_control_loop_t->gimbal_mode != CHASSIS_SPIN && 
		gimbal_control_loop_t->gimbal_mode != CHASSIS_FOLLOW_INS_YAW && 
		gimbal_control_loop_t->gimbal_mode != CHASSIS_REGULAR_YAW &&
		gimbal_control_loop_t->gimbal_mode != CHASSIS_CRUISE_NORMAL &&
		gimbal_control_loop_t->gimbal_mode != CHASSIS_CRUISE_SPIN) || gimbal_control_loop_t->aimbot_mode == AIMBOT_ON)
	{
		gimbal_control_loop_t->gimbal_yaw_motor.add = 0;		//如果不是陀螺仪控制模式底下的时候回位
		gimbal_control_loop_t->gimbal_pitch_motor.add = 0;
		gimbal_control_loop_t->ins_data.yaw_target = gimbal_control_loop_t->ins_data.yaw;	//除了小陀螺模式控制下需要对target值重复更新，防止切到陀螺仪的时候会转头
		gimbal_control_loop_t->ins_data.pitch_target = gimbal_control_loop_t->ins_data.pitch;
		gimbal_control_loop_t->ins_data.yaw_last = 0;
	}
	
	
	
	//云台控制,不同控制模式的yaw电机PID运算
	if(gimbal_control_loop_t->gimbal_mode == CHASSIS_FOLLOW_YAW || gimbal_control_loop_t->gimbal_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		gimbal_ecd_yaw_control(gimbal_control_loop_t);	//编码器控制
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_SPIN||gimbal_control_loop_t->gimbal_mode == CHASSIS_FOLLOW_INS_YAW )
	{
		gimbal_ins_yaw_control(gimbal_control_loop_t);	//陀螺仪控制
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_REGULAR_YAW)
	{
		gimbal_to_chassis_regular(gimbal_control_loop_t);
		gimbal_ins_yaw_control(gimbal_control_loop_t);	//陀螺仪控制
	}
	else if(gimbal_control_loop_t->gimbal_mode == CHASSIS_CRUISE_NORMAL || gimbal_control_loop_t->gimbal_mode == CHASSIS_CRUISE_SPIN)
	{
		gimbal_ins_yaw_control(gimbal_control_loop_t);	//陀螺仪控制
	}
	//编码器pitch电机PID运算
	pitch_ins_PID_control(gimbal_control_loop_t);
	//trigger电机PID运算
	trigger_motor_PID_control(gimbal_control_loop_t);
	//fric电机PID运算
	fric_motor_PID_control(gimbal_control_loop_t);
	
	aimbot_gimbal_control(gimbal_control_loop_t);
	
	//云台pitch轴角度环控制电流赋值
	gimbal_control_loop_t->gimbal_pitch_motor.give_current = gimbal_control_loop_t->gimbal_pitch_motor.current_set;
	//云台yaw轴角度环控制电流赋值
	gimbal_control_loop_t->gimbal_yaw_motor.give_current = gimbal_control_loop_t->gimbal_yaw_motor.current_set;
	
	//打开激光
	if(gimbal_control_loop_t->shoot_switch.fric_mode == FRIC_START)
		laser_on();
	else if(gimbal_control_loop_t->shoot_switch.fric_mode == FRIC_STOP)
		laser_off();
	
	
	gimbal_control_loop_t->gimbal_last_mode = gimbal_control_loop_t->gimbal_mode;
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_11)
	{
		gimbal_control.shoot_count += 1;
		gimbal_control.shoot_total_count += 1;
	}
	HAL_GPIO_EXTI_Callback_INS(GPIO_Pin);
}


/**
  * @brief          获取云台数据指针
  * @param[in]      none
  * @retval         获取云台数据指针
  */
const gimbal_control_t *get_gimbal_move_point(void)
{
    return &gimbal_control;
}
