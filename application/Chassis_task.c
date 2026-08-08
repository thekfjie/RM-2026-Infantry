#include "Chassis_task.h"
#include "can_receive.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "user_lib.h"
#include "steer_motor_behaviour.h"
#include "arm_math.h"
#include "bsp_delay.h"
#include "math.h"
#include "chassis_power_control.h"
#include "chassis_cruise.h"

#define CHASSIS_SPIN_SPEED		15000.0f

#define MOVE_RATE_KEY_NO_SPIN	2800.0f
#define MOVE_RATE_KEY_SPIN		3000.0f

#define CHASSIS_FOLLOW_YAW_KP			8.0f
#define CHASSIS_FOLLOW_YAW_KI			0.0f
#define CHASSIS_FOLLOW_YAW_KD			0.0f
#define CHASSIS_FOLLOW_YAW_MAX_OUT		10000.0f
#define CHASSIS_FOLLOW_YAW_MAX_IOUT		0.0f

////粗糙地板
////底盘舵向电机角度环PID
//#define M6020_MOTOR_ANGLE_PID_KP			0.16f
//#define M6020_MOTOR_ANGLE_PID_KI            0.0f
//#define M6020_MOTOR_ANGLE_PID_KD            0.0f
//#define M6020_MOTOR_ANGLE_PID_MAX_OUT       3000.0f
//#define M6020_MOTOR_ANGLE_PID_MAX_IOUT      5000.0f

////底盘舵向电机速度环PID
//#define M6020_MOTOR_SPEED_PID_KP 			280.0f
//#define M6020_MOTOR_SPEED_PID_KI 			0.0f
//#define M6020_MOTOR_SPEED_PID_KD 			12.0f
//#define M6020_MOTOR_SPEED_PID_MAX_OUT 		30000.0f
//#define M6020_MOTOR_SPEED_PID_MAX_IOUT		5000.0f

//光滑地板
//底盘舵向电机角度环PID
#define M6020_MOTOR_ANGLE_PID_KP			0.40f
#define M6020_MOTOR_ANGLE_PID_KI            0.0f
#define M6020_MOTOR_ANGLE_PID_KD            0.0f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT       30000.0f
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT      5000.0f

//底盘舵向电机速度环PID
#define M6020_MOTOR_SPEED_PID_KP 			27.0f
#define M6020_MOTOR_SPEED_PID_KI 			0.0f
#define M6020_MOTOR_SPEED_PID_KD 			0.0f
#define M6020_MOTOR_SPEED_PID_MAX_OUT 		30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT		5000.0f


chassis_move_t chassis_move;

uint32_t chassis_high_water;

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
void chassis_task(void const *pvParameters)
{
	chassis_init(&chassis_move);
	
	while(1)
	{
		chassis_set_mode(&chassis_move);
		//更新底盘数据
		chassis_feedback_update(&chassis_move);
		
		//获取三个控制设置值
		chassis_set_control(&chassis_move);
		
		//底盘运算
		chassis_control_loop(&chassis_move);
		//发送控制电流
		if(chassis_move.chassis_mode == CHASSIS_NO_MOVE)
		{			
			CAN_cmd_chassis_motor(0, 0, 0, 0);
			CAN_cmd_chassis_steer(0, 0, 0, 0);
		}
		else
		{
			CAN_cmd_chassis_motor(0, 0, 0, 0);
			CAN_cmd_chassis_steer(0, 0, 0, 0);
//			CAN_cmd_chassis_motor(chassis_move.chassis_motor[0].give_current, chassis_move.chassis_motor[1].give_current,
//									chassis_move.chassis_motor[2].give_current, chassis_move.chassis_motor[3].give_current);
//			CAN_cmd_chassis_steer(chassis_move.chassis_steer[0].give_current, chassis_move.chassis_steer[1].give_current,
//									chassis_move.chassis_steer[2].give_current, chassis_move.chassis_steer[3].give_current);
		}
		vTaskDelay(5);
#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

void steer_PID_init(steer_pid_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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

static fp32 steer_PID_calc(steer_pid_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    pid->err = set - get;
//    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
    {
        return;
    }
	
	chassis_move_init->chassis_mode = CHASSIS_NO_MOVE;	//底盘初始化无力
    //底盘速度环pid值
    const static fp32 chassis_motor_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	const static fp32 chassis_steer_speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
	const static fp32 chassis_follow_yaw_pid[3] = {CHASSIS_FOLLOW_YAW_KP, CHASSIS_FOLLOW_YAW_KI, CHASSIS_FOLLOW_YAW_KD};
	//获取遥控器指针
	chassis_move_init->chassis_RC = get_remote_control_point();
	//获取底盘指针
	chassis_move_init->chassis_gimbal_control = get_gimbal_move_point();
	//获得巡航数据
	chassis_move_init->chassis_cruise_control = get_chassis_cruise_point();
	//获取电机数据指针
    for (uint8_t i = 0; i < 4; i++)	
    {
		//获取3508电机数据
        chassis_move_init->chassis_motor[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        chassis_move_init->chassis_steer[i].chassis_motor_measure = get_chassis_steer_measure_point(i+4);
		//3508速度环
		PID_init(&chassis_move_init->chassis_motor_pid[i], PID_POSITION, chassis_motor_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
		//steer电机角度环
		steer_PID_init(&chassis_move_init->chassis_steer_angle_pid[i], M6020_MOTOR_ANGLE_PID_MAX_OUT, M6020_MOTOR_ANGLE_PID_MAX_IOUT, M6020_MOTOR_ANGLE_PID_KP, M6020_MOTOR_ANGLE_PID_KI, M6020_MOTOR_ANGLE_PID_KD);
		//steer电机速度环
		PID_init(&chassis_move_init->chassis_steer_speed_pid[i], PID_POSITION, chassis_steer_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);
	}
	PID_init(&chassis_move_init->chassis_follow_yaw_pid, PID_POSITION, chassis_follow_yaw_pid, CHASSIS_FOLLOW_YAW_MAX_OUT, CHASSIS_FOLLOW_YAW_MAX_IOUT);
}

void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
//    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_1]))
//    {
//        chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
//    }
//	else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_2]))
//	{
//		chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_YAW;
//	}
//	else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_2]))
//    {
//        chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_YAW;
//    }
//	else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_2]))
//	{
//		chassis_move_mode->chassis_mode = CHASSIS_ONLY_GIMBAL;
//	}
//	else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_2]))
//	{
//		chassis_move_mode->chassis_mode = CHASSIS_SPIN;
//	}
	chassis_move_mode->chassis_mode = chassis_move_mode->chassis_gimbal_control->gimbal_mode;
}



//纯底盘运动模式下
void chassis_move_without_follow(chassis_move_t *chassis_move_no_follow)
{
	chassis_move_no_follow->vx_set = chassis_move_no_follow->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
	chassis_move_no_follow->vy_set = -chassis_move_no_follow->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
	chassis_move_no_follow->wz_set = -chassis_move_no_follow->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
}


//底盘跟随云台模式下的运动补偿
float cos_out, sin_out, vx_set, vy_set;
void chassis_move_chassis_follow_gimbal(chassis_move_t *chassis_follow_chassis_move)
{
	if(chassis_follow_chassis_move->chassis_mode == CHASSIS_CRUISE_NORMAL ||chassis_follow_chassis_move->chassis_mode == CHASSIS_CRUISE_SPIN)
	{
		vx_set = chassis_follow_chassis_move->chassis_cruise_control->move_vx;
		vy_set = chassis_follow_chassis_move->chassis_cruise_control->move_vy;
	}
	else if(chassis_follow_chassis_move->chassis_mode == CHASSIS_SPIN)
	{
		vx_set = chassis_follow_chassis_move->chassis_RC->rc.ch[CHASSIS_X_CHANNEL]*CHASSIS_OPEN_RC_SCALE_SPIN;
		vy_set = -chassis_follow_chassis_move->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]*CHASSIS_OPEN_RC_SCALE_SPIN;

		vx_set +=  chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].w *MOVE_RATE_KEY_SPIN
							  - chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].s *MOVE_RATE_KEY_SPIN;
		vy_set +=  - chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].a *MOVE_RATE_KEY_SPIN
							  + chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].d *MOVE_RATE_KEY_SPIN;
	}
	else
	{
		vx_set = chassis_follow_chassis_move->chassis_RC->rc.ch[CHASSIS_X_CHANNEL]*CHASSIS_OPEN_RC_SCALE;
		vy_set = -chassis_follow_chassis_move->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]*CHASSIS_OPEN_RC_SCALE;
		
		vx_set +=  chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].w * MOVE_RATE_KEY_NO_SPIN
					  - chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].s * MOVE_RATE_KEY_NO_SPIN;
		vy_set +=  - chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].a * MOVE_RATE_KEY_NO_SPIN
							  + chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].d * MOVE_RATE_KEY_NO_SPIN;
	}
	
	if(chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].shift == 1)
	{
		vx_set *= 3.0f;
		vy_set *= 3.0f;
	}
	
	chassis_follow_chassis_move->err_angle = chassis_follow_chassis_move->chassis_gimbal_control->yaw*MOTOR_RAD_TO_ECD*MOTOR_RAD;
	chassis_follow_chassis_move->err_angle += chassis_follow_chassis_move->chassis_gimbal_control->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm*ERR_ANGLE_COMPENSATE;
	chassis_follow_chassis_move->err_angle -= CHASSIS_ERR_ECD;
	cos_out = arm_cos_f32(-chassis_follow_chassis_move->err_angle);
	sin_out = arm_sin_f32(-chassis_follow_chassis_move->err_angle);
	chassis_follow_chassis_move->vx_set = cos_out * vx_set + sin_out * vy_set;
	chassis_follow_chassis_move->vy_set = -sin_out * vx_set + cos_out * vy_set;

	
	if(chassis_follow_chassis_move->chassis_mode == CHASSIS_FOLLOW_YAW)
		{
			chassis_follow_chassis_move->wz_set = PID_calc(&chassis_follow_chassis_move->chassis_follow_yaw_pid, 0.0f, chassis_follow_chassis_move->chassis_gimbal_control->yaw);

		}
		else if(chassis_follow_chassis_move->chassis_mode == CHASSIS_FOLLOW_INS_YAW)
		{
			//这个pid的位置不是很好，因为在云台写错了relative_angle和absolute_angle
			chassis_follow_chassis_move->wz_set = PID_calc(&chassis_follow_chassis_move->chassis_follow_yaw_pid, 0.0f, chassis_follow_chassis_move->chassis_gimbal_control->yaw);
		}
//	if(fabs(chassis_follow_chassis_move->chassis_gimbal_control->yaw)<GIMBAL_CHASSIS_STEER_SEN&&chassis_follow_chassis_move->chassis_mode != CHASSIS_SPIN)
//	{
//		if(chassis_follow_chassis_move->chassis_gimbal_control->yaw>=0)
//			chassis_follow_chassis_move->wz_set = 500.0f;
//		else if(chassis_follow_chassis_move->chassis_gimbal_control->yaw<0)
//			chassis_follow_chassis_move->wz_set = -500.0f;
//	}
	static uint8_t chassis_spin_state;
	if(chassis_follow_chassis_move->chassis_mode == CHASSIS_SPIN)
	{
		if(chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].ctrl == 1&&chassis_follow_chassis_move->wz_set>0.01f&&chassis_spin_state==0)
		{
			chassis_follow_chassis_move->wz_set = -CHASSIS_SPIN_SPEED;
			chassis_spin_state =1;
		}
		else if(chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].ctrl == 1&&chassis_follow_chassis_move->wz_set<-0.01f&&chassis_spin_state==0)
		{
			chassis_follow_chassis_move->wz_set = CHASSIS_SPIN_SPEED;
			chassis_spin_state =1;
		}
		else if(chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].ctrl == 0&&fabs(chassis_follow_chassis_move->wz_set)<CHASSIS_SPIN_SPEED)
			chassis_follow_chassis_move->wz_set = CHASSIS_SPIN_SPEED;
		else if(chassis_follow_chassis_move->chassis_RC->key[KEY_PRESS].ctrl == 0)
			chassis_spin_state = 0;
	}
}

// 小陀螺模式
void chassis_move_spin(chassis_move_t *chassis_spin)
{
	chassis_move_chassis_follow_gimbal(chassis_spin);
	
}
//底盘固定模式
void chassis_regular_move(chassis_move_t *chassis_regular)
{
	chassis_move_chassis_follow_gimbal(chassis_regular);
	chassis_regular->wz_set = 0;
}

void chassis_set_control(chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL)
    {
        return;
    }
	//底盘运动赋值
	if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		chassis_move_without_follow(chassis_move_rc_to_vector);	//纯底盘运动模式下
	}
	else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_YAW)
	{
		chassis_move_chassis_follow_gimbal(chassis_move_rc_to_vector);	//底盘跟随云台模式下的运动补偿
	}
	else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_INS_YAW)
	{
		chassis_move_chassis_follow_gimbal(chassis_move_rc_to_vector);	//底盘跟随云台模式下的运动补偿
	}
	else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_CRUISE_NORMAL)
	{
		chassis_regular_move(chassis_move_rc_to_vector);	//底盘跟随云台模式下的运动补偿
	}
	else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_SPIN)
	{
		chassis_move_spin(chassis_move_rc_to_vector);		//小陀螺模式下的运动补偿
	}
	else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_CRUISE_SPIN)
	{
		chassis_move_spin(chassis_move_rc_to_vector);		//小陀螺模式下的运动补偿
	}
	else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_REGULAR_YAW)
	{
		chassis_regular_move(chassis_move_rc_to_vector);
	}
	
    return;
}

void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
		//3508
        chassis_move_update->chassis_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->chassis_motor[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->chassis_motor[i].accel = chassis_move_update->chassis_motor_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		//6020
		chassis_move_update->chassis_steer[i].speed = chassis_move_update->chassis_steer[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->chassis_steer[i].absolute_angle = chassis_move_update->chassis_steer[i].chassis_motor_measure->ecd;
	}
	
	chassis_move_update->chassis_state = CHASSIS_STATE_NORMAL;
    for (uint8_t i = 0; i < 4; i++)
    {
		if(chassis_move_update->chassis_steer[i].chassis_motor_measure->temperate == 0&&chassis_move_update->chassis_motor[i].chassis_motor_measure->temperate == 0)
		{
			chassis_move_update->chassis_state = CHASSIS_ALL_TOE;
			break;
		}
		if(chassis_move_update->chassis_steer[i].chassis_motor_measure->temperate == 0)
		{
			chassis_move_update->chassis_state = CHASSIS_STEER_TOE;
			break;
		}
		else if(chassis_move_update->chassis_motor[i].chassis_motor_measure->temperate == 0)
		{
			chassis_move_update->chassis_state = CHASSIS_MOTOR_TOE;
			break;
		}
	}
}

//过零点判断
fp32 motor_ecd_to_angle_change(int16_t target_ecd, int16_t real_ecd)
{
    if (target_ecd >= ECD_RANGE)
    {
        target_ecd -= ECD_RANGE;
    }
    else if (target_ecd < 0)
    {
        target_ecd += ECD_RANGE;
    }

    int16_t diff = real_ecd - target_ecd;
    if (diff > HALF_ECD_RANGE)
    {
        target_ecd += ECD_RANGE;
    }
    else if (diff < -HALF_ECD_RANGE)
    {
        target_ecd -= ECD_RANGE;
    }
	
    return target_ecd;
}

//舵向最优角度换算
static int16_t dirt[4] = { 1, 1, -1, -1};
static int16_t change[4] = { 1, 1, -1, -1};
int16_t diff = 0;
fp32 motor_ecd_to_angle_change_half(int16_t target_ecd, int16_t real_ecd, uint8_t i)
{
	if (target_ecd >= ECD_RANGE)
    {
        target_ecd -= ECD_RANGE;
    }
    else if (target_ecd < 0)
    {
        target_ecd += ECD_RANGE;
    }
	
	diff = real_ecd - target_ecd;
    if (diff >= QUAR_EDC_RANGE)
    {
        target_ecd += HALF_ECD_RANGE;
		if(dirt[i] == change[i])
			dirt[i] = -dirt[i];
    }
    else if (diff < -QUAR_EDC_RANGE)
    {
        target_ecd -= HALF_ECD_RANGE;
		if(dirt[i] == change[i])
			dirt[i] = -dirt[i];
    }
	else
	{
		dirt[i] = change[i];
	}
    return target_ecd;
}


static void chassis_vector_to_M3508_wheel_speed(chassis_move_t *chassis_motor, fp32 vx_set, fp32 vy_set, fp32 wz_set)
{
	fp32 wheel_rpm_ratio;
	
    wheel_rpm_ratio = 0.04f / (WHEEL_PERIMETER * PI) * M3508_RATIO * 0.05f;	
	
    chassis_motor->chassis_motor[0].speed_set = dirt[0] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_motor->chassis_motor[1].speed_set = dirt[1] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_motor->chassis_motor[2].speed_set = dirt[2] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set + wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_motor->chassis_motor[3].speed_set = dirt[3] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set + wz_set * Radius * 0.707107f,2) 
                       ) * wheel_rpm_ratio ;
		
}


//舵轮运算
static void chassis_steer_motor_count(chassis_move_t *chassis_steer_motor, fp32 vx_set, fp32 vy_set, fp32 wz_set)
{
	float trun_rate;
	if(chassis_steer_motor->chassis_mode == CHASSIS_SPIN)
		trun_rate = Spin_rate;
	else
		trun_rate = Radius;
    chassis_steer_motor->chassis_steer[0].absolute_angle_set = atan2((vx_set - wz_set * trun_rate * 0.707107f),(vy_set + wz_set * trun_rate * 0.707107f)) * 180.0f / PI+180;        
    chassis_steer_motor->chassis_steer[1].absolute_angle_set = atan2((vx_set + wz_set * trun_rate * 0.707107f),(vy_set + wz_set * trun_rate * 0.707107f)) * 180.0f / PI+180;
    chassis_steer_motor->chassis_steer[2].absolute_angle_set = atan2((vx_set - wz_set * trun_rate * 0.707107f),(vy_set - wz_set * trun_rate * 0.707107f)) * 180.0f / PI+180;
    chassis_steer_motor->chassis_steer[3].absolute_angle_set = atan2((vx_set + wz_set * trun_rate * 0.707107f),(vy_set - wz_set * trun_rate * 0.707107f)) * 180.0f / PI+180;    

	
	chassis_steer_motor->chassis_steer[0].target_ecd = motor_ecd_to_angle_change_half(chassis_steer_motor->chassis_steer[0].absolute_angle_set*MOTOR_ECD_TO_RAD + M6020_CHASSIS_ECD_1, chassis_steer_motor->chassis_steer[0].absolute_angle, 0);
	chassis_steer_motor->chassis_steer[1].target_ecd = motor_ecd_to_angle_change_half(chassis_steer_motor->chassis_steer[1].absolute_angle_set*MOTOR_ECD_TO_RAD + M6020_CHASSIS_ECD_2, chassis_steer_motor->chassis_steer[1].absolute_angle, 1);
	chassis_steer_motor->chassis_steer[2].target_ecd = motor_ecd_to_angle_change_half(chassis_steer_motor->chassis_steer[2].absolute_angle_set*MOTOR_ECD_TO_RAD + M6020_CHASSIS_ECD_3, chassis_steer_motor->chassis_steer[2].absolute_angle, 2);
	chassis_steer_motor->chassis_steer[3].target_ecd = motor_ecd_to_angle_change_half(chassis_steer_motor->chassis_steer[3].absolute_angle_set*MOTOR_ECD_TO_RAD + M6020_CHASSIS_ECD_4, chassis_steer_motor->chassis_steer[3].absolute_angle, 3);
	
	

}




void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	if (chassis_move_control_loop == NULL)
    {
        return;
    }
	
	//舵轮3508运算
	chassis_vector_to_M3508_wheel_speed(chassis_move_control_loop, chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set);
	//舵轮6020运算
	chassis_steer_motor_count(chassis_move_control_loop, chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set);
	
	
	
	//PID运算
	for(int i = 0; i<4; i++)
	{
		//3508速度环PID运算
		chassis_move_control_loop->chassis_motor[i].current_set = PID_calc(&chassis_move_control_loop->chassis_motor_pid[i], chassis_move_control_loop->chassis_motor[i].speed, chassis_move_control_loop->chassis_motor[i].speed_set);
		//6020
		chassis_move_control_loop->chassis_steer[i].motor_gyro_set = steer_PID_calc(&chassis_move_control_loop->chassis_steer_angle_pid[i], chassis_move_control_loop->chassis_steer[i].absolute_angle, chassis_move_control_loop->chassis_steer[i].target_ecd, chassis_move_control_loop->chassis_steer[i].speed);
		chassis_move_control_loop->chassis_steer[i].current_set = PID_calc(&chassis_move_control_loop->chassis_steer_speed_pid[i], chassis_move_control_loop->chassis_steer[i].speed, chassis_move_control_loop->chassis_steer[i].motor_gyro_set);
    }
	
	chassis_power_control(chassis_move_control_loop);
	
	//电流赋值
	for(int i = 0; i<4; i++)
	{
		chassis_move_control_loop->chassis_motor[i].give_current = chassis_move_control_loop->chassis_motor[i].current_set;
		chassis_move_control_loop->chassis_steer[i].give_current = chassis_move_control_loop->chassis_steer[i].current_set;
	}
	
	chassis_move_control_loop->chassis_last_mode = chassis_move_control_loop->chassis_mode;
}


/**
  * @brief          获取底盘数据指针
  * @param[in]      none
  * @retval         获取底盘数据指针
  */
const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}
