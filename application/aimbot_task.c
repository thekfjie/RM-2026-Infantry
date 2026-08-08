#include "aimbot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "bsp_usart.h"
#include "string.h"
#include "usart.h"
#include "crc8_crc16.h"
#include "AHRS.H"
#include "user_lib.h"
#include "arm_math.h"
#include "aimbot_solving.h"
#include "arm_constrain.h"

//自瞄系统延时ms
#define AIMBOT_TASK_TIME	  40 //15
//z轴高度补偿，弥补上位URDF误差，偏高减小
#define Z_BUFFER	 		-0.10f
//yaw
#define YAW_BUFFER      -0.003f   //-0.003f

//自瞄判断小陀螺和不小陀螺的阈值
//#define STAND_SPIN_THRE		0.0065f

//#define AIMBOT_SPIN_BUFFER	

//延时单位 秒
//如果子弹偏后，增大这个值
#define DELAY_BUFFER		-0.04f   //0.02f      1. 旋转目标打偏 “后方”（旋转方向的反方向），增大该值；2. 打偏 “前方”，减小该值；3. 初始 0.02，步长 0.005
#define COMMUNICATION_DELAY	0.04f				//通信延迟

#define SPIN_BUFFER			0.0f
#define LINE_BUFFER			0.0f

#define FRICTION_K			0.001793f		//空气摩擦因素
#define SMALL_PELLETS		0.0032f			//小弹丸质量3.2g
#define PELLETS_VELOCITY	26.0f	//弹丸初速度

#define STAND_SPIN_THRE   0.1f
#define SPIN_DIR_THRE    0.05f
#define SPIN_RADIUS_OFFSET    -0.002f  //1. 打静止旋转目标，若总是打偏外侧，增大该值；2. 打偏内侧，减小该值；3. 初始值 0.01（1cm），步长 0.005

#define SPIN_ANGLE_LIMIT    PI/3
#define SPIN_TARGET_PRIORITY 1
#define LINE_TARGET_PRIORITY 0

aimbot_task_t aimbot;

uint8_t AIMBOT_LEN_TX;
uint8_t AIMBOT_LEN_RX;
uint8_t aimbot_tx_buffer[28];	//发送给视觉的数组
uint8_t aimbot_rx_buffer[48];	//接收到视觉的数组

//串口接收
static uint8_t RxCounter;
static uint8_t rxd_state = 0;
uint8_t aimbot_data;

uint8_t x0x0;
uint8_t y0y0;



////子弹飞行时间计算  试运行XJL版本  太难调了，还是用LZX
//void aimbot_flight_time_count(aimbot_count_t *aimbot_count)
//{
//    fp32 distance_t = sqrt(pow(aimbot.target_x,2)+pow(aimbot.target_y,2));
//    fp32 velocity_t = sqrt(pow(aimbot.Vision_Rx_Data.vx,2)+pow(aimbot.Vision_Rx_Data.vy,2));
//    
//    // 旋转目标时，加入角速度对距离的修正
//    fp32 spin_vel = fabs(aimbot.Vision_Rx_Data.v_yaw);
//    if(spin_vel > STAND_SPIN_THRE)
//    {
//        // 旋转目标的等效平动速度（半径×角速度）
//        fp32 r_average = (aimbot.Vision_Rx_Data.r1+aimbot.Vision_Rx_Data.r2)/2.0f + SPIN_RADIUS_OFFSET;
//        velocity_t = r_average * spin_vel;
//    }
//    
//    aimbot_count->t_out = 0.0f; // 初始化
//    for(int i=0; i<20; i++) 
//    {
//        aimbot_count->t0 = (distance_t)/PELLETS_VELOCITY;
//        aimbot_count->ft1 = (SMALL_PELLETS/FRICTION_K)*log((FRICTION_K*PELLETS_VELOCITY/SMALL_PELLETS)*aimbot_count->t0+1)
//                            -distance_t-velocity_t*aimbot_count->t0;
//        aimbot_count->ft2 = PELLETS_VELOCITY/((FRICTION_K*PELLETS_VELOCITY)*aimbot_count->t0+1) - velocity_t;
//        
//        // 避免除零错误
//        if(fabs(aimbot_count->ft2) < 1e-6)
//        {
//            break;
//        }
//        
//        aimbot_count->t1 = aimbot_count->t0 - (aimbot_count->ft1/aimbot_count->ft2);
//        
//        // 收敛判断（误差小于1e-5时退出）
//        if(fabs(aimbot_count->t1 - aimbot_count->t0) < 1e-5)
//        {
//            aimbot_count->t_out = COMMUNICATION_DELAY + aimbot_count->t1 + DELAY_BUFFER;
//            break;
//        }
//        aimbot_count->t0 = aimbot_count->t1; // 更新迭代值
//    }
//    //若迭代未收敛，用基础值
//    if(aimbot_count->t_out == 0.0f)
//    {
//        aimbot_count->t_out = COMMUNICATION_DELAY + (distance_t/PELLETS_VELOCITY) + DELAY_BUFFER;
//    }
//}


//子弹飞行时间计算  LZX版本
void aimbot_flight_time_count(aimbot_count_t *aimbot_count)
{
	fp32 distance_t = sqrt(pow(aimbot.target_x,2)+pow(aimbot.target_y,2));
	fp32 velocity_t = sqrt(pow(aimbot.Vision_Rx_Data.vx,2)+pow(aimbot.Vision_Rx_Data.vy,2));
	for(int i=0; i<50; i++)
	{
		aimbot_count->t0 = (distance_t)/PELLETS_VELOCITY;
		aimbot_count->ft1 = (SMALL_PELLETS/FRICTION_K)*log((FRICTION_K*PELLETS_VELOCITY/SMALL_PELLETS)*aimbot_count->t0+1)
								-distance_t-velocity_t*aimbot_count->t0;
		aimbot_count->ft2 = PELLETS_VELOCITY/((FRICTION_K*PELLETS_VELOCITY)*aimbot_count->t0+1) - velocity_t;
		aimbot_count->t1 = aimbot_count->t0 - (aimbot_count->ft1/aimbot_count->ft2);
		if(fabs(aimbot_count->t1 - aimbot_count->t0) > 1e-5)
		{
			aimbot_count->t_out = (COMMUNICATION_DELAY + aimbot_count->t1 +DELAY_BUFFER);
//			aimbot_count->t_out = 0.0f;
		}
	}
}


static uint8_t aimbot_mode_state = 0;
void aimbot_task(void const *pvParameters)
{
	vTaskDelay(1000);
	aimbot_init(&aimbot);
	while(1)
	{
		aimbot_flight_time_count(&aimbot.aimbot_count);
		aimbot_feedback_update(&aimbot);
		aimbot_control_loop(&aimbot);
		
		HAL_UART_Transmit_IT(&huart1, aimbot_tx_buffer, sizeof(aimbot.Vision_Tx_Data));

		vTaskDelay(AIMBOT_TASK_TIME);
	}
}




void aimbot_init(aimbot_task_t *aimbot_task_init)
{

	//陀螺仪数据指针获取
	aimbot_task_init->aimbot_INT_angle_point = get_INS_angle_point();
	aimbot_task_init->aimbot_INT_gyro_point = get_gyro_data_point();
	aimbot_task_init->aimbot_INT_quat_point = get_INS_quat_point();
  
  
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&aimbot_data, 1);
	AIMBOT_LEN_TX = (uint16_t)(sizeof(aimbot.Vision_Tx_Data));
	AIMBOT_LEN_RX = (uint16_t)(sizeof(aimbot.Vision_Rx_Data));
  
	aimbot_task_init->Vision_Tx_Data.header = 0x5A;
	aimbot_task_init->Vision_Tx_Data.detect_color = Get_ID();
//	aimbot_task_init->Vision_Tx_Data.detect_color = 0;  //打红
//	aimbot_task_init->Vision_Tx_Data.detect_color = 1;  //打蓝
	aimbot_task_init->Vision_Tx_Data.reset_tracker = 1;
	aimbot_task_init->Vision_Tx_Data.reserved = 0;
	
	
	append_CRC16_check_sum((uint8_t *)&aimbot_task_init->Vision_Tx_Data, sizeof(aimbot.Vision_Tx_Data));
  
  
	memcpy(aimbot_tx_buffer, &aimbot.Vision_Tx_Data, sizeof(aimbot.Vision_Tx_Data));  //传入需要发送的数据
	HAL_UART_Transmit_IT(&huart1, aimbot_tx_buffer, sizeof(aimbot.Vision_Tx_Data));
	aimbot_task_init->Vision_Tx_Data.reset_tracker = 0;
}

//自瞄数据指针获取
void aimbot_feedback_update(aimbot_task_t *aimbot_feedback)
{
	aimbot_feedback->Vision_Tx_Data.detect_color = Get_ID();
	if(aimbot_feedback->Vision_Rx_Data.x != aimbot_feedback->last_x)
		aimbot_feedback->aimbot_state = AIMBOT_GET;
	else
//		if(aimbot_feedback->aimbot_mode == AIMBOT_OFF)
			aimbot_feedback->aimbot_state = AIMBOT_OUT;
	
	aimbot_feedback->last_x = aimbot_feedback->Vision_Rx_Data.x;
	aimbot_feedback->last_y = aimbot_feedback->Vision_Rx_Data.y;
	if(aimbot_feedback->aimbot_mode == AIMBOT_RESET && aimbot_mode_state == 0)
	{
		aimbot_feedback->Vision_Tx_Data.reset_tracker = 1;
		aimbot_mode_state = 1;
	}
	else if(aimbot_mode_state == 1)
	{
		aimbot_feedback->Vision_Tx_Data.reset_tracker = 0;
	}
	if(aimbot_feedback->aimbot_mode == AIMBOT_OFF)
	{
		aimbot_mode_state = 0;
	}
//	
//	aimbot_feedback->vx = (aimbot_feedback->Vision_Rx_Data.x - aimbot_feedback->last_x) * 50;
//	aimbot_feedback->vy = (aimbot_feedback->Vision_Rx_Data.y - aimbot_feedback->last_y) * 50;
/******************************************发送不用改*******************************************/
	aimbot_feedback->Vision_Tx_Data.yaw = 		aimbot_feedback->aimbot_INT_angle_point[YAW_CHANNEL];
	aimbot_feedback->Vision_Tx_Data.pitch = 	-aimbot_feedback->aimbot_INT_angle_point[PITCH_CHANNEL];
	aimbot_feedback->Vision_Tx_Data.roll = 		aimbot_feedback->aimbot_INT_angle_point[ROLL_CHANNEL];
		
	append_CRC16_check_sum((uint8_t *)&aimbot_feedback->Vision_Tx_Data, sizeof(aimbot.Vision_Tx_Data));
  
	memcpy(aimbot_tx_buffer, &aimbot.Vision_Tx_Data, sizeof(aimbot.Vision_Tx_Data));  //传入需要发送的数据
	
//LZX
	fp32 r_average = (aimbot_feedback->Vision_Rx_Data.r1+aimbot_feedback->Vision_Rx_Data.r2)/2.0f;
//	fp32 r_average = (aimbot_feedback->Vision_Rx_Data.r1+aimbot_feedback->Vision_Rx_Data.r2)/2.0f+SPIN_RADIUS_OFFSET;
	fp32 spin_vel = aimbot_feedback->Vision_Rx_Data.v_yaw; // 保留角速度符号（判断旋转方向）
	fp32 spin_vel_abs = fabs(spin_vel);
	//---------------------------------------------------------------------------------
	// 旋转目标标记
//uint8_t is_spin_target = (spin_vel_abs > STAND_SPIN_THRE) ? SPIN_TARGET_PRIORITY : LINE_TARGET_PRIORITY;

//if(is_spin_target)
//{
//    // 计算飞行时间内目标旋转的总角度
//    fp32 spin_angle = spin_vel * aimbot_feedback->aimbot_count.t_out;
//    // 限制单次补偿角度，避免超过120°（防止视觉跳变导致的错误补偿）
////    spin_angle = arm_constrain(spin_angle, -SPIN_ANGLE_LIMIT, SPIN_ANGLE_LIMIT);
//    
//    // 计算目标中心的实时位置（视觉给出的x/y是底盘中心，需修正到装甲板位置）
//    fp32 target_center_x = aimbot_feedback->Vision_Rx_Data.x;
//    fp32 target_center_y = aimbot_feedback->Vision_Rx_Data.y;
//    fp32 current_armor_angle = aimbot_feedback->Vision_Rx_Data.yaw; // 当前装甲板角度
//    
//    // 预测飞行时间后装甲板的角度（当前角度 + 旋转角度）
//    fp32 predict_armor_angle = current_armor_angle + spin_angle;
//    
//    // 计算预测的装甲板位置
//    aimbot_feedback->target_x = target_center_x + r_average * cos(predict_armor_angle);
//    aimbot_feedback->target_y = target_center_y + r_average * sin(predict_armor_angle);
//    
//    // 关闭平动补偿（旋转目标优先，平动补偿会干扰）
//    aimbot_feedback->target_x -= aimbot_feedback->Vision_Rx_Data.vx * aimbot_feedback->aimbot_count.t_out;
//    aimbot_feedback->target_y -= aimbot_feedback->Vision_Rx_Data.vy * aimbot_feedback->aimbot_count.t_out;
//}
//else
//{
//    // 非旋转目标，保留原平动补偿逻辑（可微调）
//    aimbot_feedback->target_x = aimbot_feedback->Vision_Rx_Data.x 
//        + aimbot_feedback->Vision_Rx_Data.vx * (aimbot_feedback->aimbot_count.t_out + LINE_BUFFER);
//    aimbot_feedback->target_y = aimbot_feedback->Vision_Rx_Data.y 
//        + aimbot_feedback->Vision_Rx_Data.vy * (aimbot_feedback->aimbot_count.t_out + LINE_BUFFER);
//}

//// Z轴补偿保留，但新增旋转目标的Z轴修正
//if(is_spin_target)
//{
//    // 旋转目标的Z轴可能有微小波动，增加补偿
//    aimbot_feedback->target_z = projectile_solve(sqrtf(x0x0+y0y0), 
//        aimbot_feedback->Vision_Rx_Data.z - aimbot_feedback->Vision_Rx_Data.dz*0.1f - 0.02f, // 新增0.02f旋转Z补偿
//        aimbot_feedback->aimbot_count.t_out) + Z_BUFFER;
//}
//else
//{
//    aimbot_feedback->target_z = projectile_solve(sqrtf(x0x0+y0y0), 
//        aimbot_feedback->Vision_Rx_Data.z - aimbot_feedback->Vision_Rx_Data.dz*0.1f, 
//        aimbot_feedback->aimbot_count.t_out) + Z_BUFFER;
//}
	//------------------------------------------------------------------------------
	if(fabs(aimbot_feedback->Vision_Rx_Data.v_yaw)>6.0f)
	{
		r_average = 0;
	}
	else if(fabs(aimbot_feedback->Vision_Rx_Data.v_yaw)>4.5f)
	{
		if(fabs(aimbot_feedback->Vision_Rx_Data.v_yaw*aimbot_feedback->aimbot_count.t_out+aimbot_feedback->Vision_Rx_Data.yaw)>PI/4.0f)
		{
			aimbot_feedback->Vision_Rx_Data.yaw -= PI/2.0f;
		}
	}
	
	

	//预测和重力补偿的运算
//	if(fabs(aimbot_feedback->Vision_Rx_Data.v_yaw)<STAND_SPIN_THRE)
//	{
		aimbot_feedback->target_x = aimbot_feedback->Vision_Rx_Data.x - r_average*cos(aimbot_feedback->Vision_Rx_Data.yaw+aimbot_feedback->Vision_Rx_Data.v_yaw*aimbot_feedback->aimbot_count.t_out)+aimbot_feedback->Vision_Rx_Data.vx*(aimbot_feedback->aimbot_count.t_out+LINE_BUFFER);
		aimbot_feedback->target_y = aimbot_feedback->Vision_Rx_Data.y - r_average*sin(aimbot_feedback->Vision_Rx_Data.yaw+aimbot_feedback->Vision_Rx_Data.v_yaw*aimbot_feedback->aimbot_count.t_out)+aimbot_feedback->Vision_Rx_Data.vy*(aimbot_feedback->aimbot_count.t_out+LINE_BUFFER);
//	}
//	else
//	{
//		aimbot_feedback->target_x = aimbot_feedback->Vision_Rx_Data.x;
//		aimbot_feedback->target_y = aimbot_feedback->Vision_Rx_Data.y;
//	}
//	aimbot_feedback->temple_z = atan(aimbot_feedback->Vision_Rx_Data.z/sqrt(pow(aimbot_feedback->target_x,2)+pow(aimbot_feedback->target_y,2)));
	
	fp32 x0x0 = aimbot_feedback->target_x*aimbot_feedback->target_x;
	fp32 y0y0 = aimbot_feedback->target_y*aimbot_feedback->target_y;
	aimbot_feedback->target_z = projectile_solve(sqrtf(x0x0+y0y0), aimbot_feedback->Vision_Rx_Data.z-aimbot_feedback->Vision_Rx_Data.dz*0.1f, aimbot_feedback->aimbot_count.t_out) + Z_BUFFER;
//	aimbot_feedback->target_z = aimbot_feedback->Vision_Rx_Data.z + Z_BUFFER;
	
}

uint8_t aimbot_auto_fire_control(fp32 yaw, fp32 target_yaw)
{
	if(fabs(yaw - target_yaw)<AIMBOT_AUTO_FIRE_THRE)
		return AIMBOT_FIRE_ON;
	else
		return AIMBOT_FIRE_OFF;
}

//自瞄模式切换
void aimbot_mode_set(uint8_t aimbot_mode)
{
	aimbot.aimbot_mode = aimbot_mode;
}




//fp32 quar[4];
void aimbot_quar_to_angle_count(aimbot_task_t *aimbot_quar_to_angle)
{
//	quar[1] = aimbot_quar_to_angle->target_x;
//	quar[2] = aimbot_quar_to_angle->target_y;
  //传出的时候也对yaw和roll轴调换了一下位置
//  aimbot_quar_to_angle->target_yaw = get_yaw(quar);
//  aimbot_quar_to_angle->target_roll = get_roll(quar);
//  aimbot_quar_to_angle->target_pitch = get_pitch(quar);
	fp32 a_yaw_rad;
	fp32 a_pitch_rad;
	a_yaw_rad = aimbot_yaw_world_to_rad(aimbot_quar_to_angle->target_x, aimbot_quar_to_angle->target_y, aimbot_quar_to_angle->target_z);
	a_pitch_rad = aimbot_pitch_world_to_rad(aimbot_quar_to_angle->target_x, aimbot_quar_to_angle->target_y, aimbot_quar_to_angle->target_z);
	aimbot_quar_to_angle->target_yaw = a_yaw_rad+YAW_BUFFER;
	aimbot_quar_to_angle->target_pitch = a_pitch_rad;
}



//自瞄计算
void aimbot_control_loop(aimbot_task_t *aimbot_control)
{
	aimbot_quar_to_angle_count(aimbot_control);

	//如果没有识别到的时候不复制
	if(	aimbot_control->Vision_Rx_Data.x==0&&aimbot_control->Vision_Rx_Data.y==0&&
		aimbot_control->Vision_Rx_Data.z==0&&aimbot_control->Vision_Rx_Data.yaw==0)
	{
		aimbot_control->target_pitch = aimbot_control->aimbot_INT_angle_point[PITCH_CHANNEL];
		aimbot_control->target_roll =  aimbot_control->aimbot_INT_angle_point[ROLL_CHANNEL];
		aimbot_control->target_yaw =   aimbot_control->aimbot_INT_angle_point[YAW_CHANNEL];
	}
}

//---------------------------------------------------------
//void aimbot_control_loop(aimbot_task_t *aimbot_control)
//{
//    aimbot_quar_to_angle_count(aimbot_control);

//    // 优化：更鲁棒的目标识别判断（避免0值误判）
//    fp32 target_valid = fabs(aimbot_control->Vision_Rx_Data.x) + fabs(aimbot_control->Vision_Rx_Data.y) + fabs(aimbot_control->Vision_Rx_Data.z);
//    if(target_valid < 0.01f) // 阈值设0.01，避免浮点精度问题
//    {
//        // 未识别到目标时，保持当前角度（原逻辑保留）
//        aimbot_control->target_pitch = aimbot_control->aimbot_INT_angle_point[PITCH_CHANNEL];
//        aimbot_control->target_roll =  aimbot_control->aimbot_INT_angle_point[ROLL_CHANNEL];
//        aimbot_control->target_yaw =   aimbot_control->aimbot_INT_angle_point[YAW_CHANNEL];
//    }
//    else
//    {
//        // 新增：旋转目标时，角度补偿平滑处理
//        fp32 spin_vel_abs = fabs(aimbot_control->Vision_Rx_Data.v_yaw);
//        if(spin_vel_abs > STAND_SPIN_THRE)
//        {
//            // 角度滤波：避免旋转目标角度跳变
//            static fp32 last_target_yaw = 0.0f;
//            aimbot_control->target_yaw = 0.8f * last_target_yaw + 0.2f * aimbot_control->target_yaw;
//            last_target_yaw = aimbot_control->target_yaw;
//        }
//    }
//}
//-------------------------------------------------------------


//串口接收中断
void aimbot_receive_program(void)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aimbot_data, 1);//完成一次接受，再此开启中断
    switch(rxd_state)
    {
      case 0:
      {
        if(aimbot_data == 0xA5)   //收到
        {
            RxCounter = 0; // 重置 RxCounter 为 0
            rxd_state = 1;
            aimbot_rx_buffer[RxCounter]=aimbot_data;
            RxCounter++;
        }
        break;
      }
      case 1:
      {
        if(aimbot_data == 0xA5)   //如果再次收到0xA5，重新开始接收
        {
            rxd_state = 0;
//            memset(&aimbot.Vision_Rx_Data, 0, sizeof(aimbot.Vision_Rx_Data)); //原始数据清除
            break;
        }
        aimbot_rx_buffer[RxCounter]=aimbot_data;
        RxCounter++;
        if(RxCounter==sizeof(aimbot.Vision_Rx_Data))  //收满
            rxd_state = 2;
        break;
      }
      case 2:
      {
        if(verify_CRC16_check_sum((uint8_t *)&aimbot_rx_buffer, sizeof(aimbot_rx_buffer)) == true)
        {
          memcpy(&aimbot.Vision_Rx_Data, aimbot_rx_buffer, sizeof(aimbot_rx_buffer)); //传出接收到的数据
//		  aimbot_mode_set(0);
        }
        rxd_state = 0;
        break;
      }
    }
}

//试运行

///-----------------------------------------------
//// 判断旋转方向（顺时针/逆时针）
//uint8_t spin_dir = (spin_vel > SPIN_DIR_THRE) ? 1 : ((spin_vel < -SPIN_DIR_THRE) ? -1 : 0);
//// 补偿时乘以方向，避免角度计算错误
//fp32 predict_armor_angle = current_armor_angle + spin_dir * fabs(spin_angle);

//static fp32 last_v_yaw = 0.0f;
//aimbot_feedback->Vision_Rx_Data.v_yaw = 0.7f * last_v_yaw + 0.3f * aimbot_feedback->Vision_Rx_Data.v_yaw;
//last_v_yaw = aimbot_feedback->Vision_Rx_Data.v_yaw;

//// 选择r1/r2中更小的（更靠近我方的装甲板）
//r_average = fmin(aimbot_feedback->Vision_Rx_Data.r1, aimbot_feedback->Vision_Rx_Data.r2) + SPIN_RADIUS_OFFSET;

///-----------------------------------------------------




/**
  * @brief          获取底盘数据指针
  * @param[in]      none
  * @retval         获取底盘数据指针
  */
const aimbot_task_t *get_aimbot_point(void)
{
    return &aimbot;
}
