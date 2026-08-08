#include "oled_task.h"
#include "main.h"
#include "oled.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "cmsis_os.h"

#define OLED_START_TIME 100
#define OLED_CONTROL_TIME 10
#define OLED_TASK_START_DELAY 1000

//屏幕分辨率
#define CHASSIS_MODE_NAME					"chassis_mode"
#define ALIGNMENT_LINE_X_NAME					"x"
#define ALIGNMENT_LINE_Y_NAME					"y"

#define ALIGNMENT_WIDE						60
#define ALIGNMENT_CENTER_X				930	   //960
#define ALIGNMENT_CENTER_Y					455

static uint16_t chassis_mode_circle_position_x=	   960;
#define CHASSIS_MODE_CIRCLE_POSITION_Y		455
#define CHASSIS_MODE_CIRCLE_R				5
#define CHASSIS_MODE_CIRCLE_LINE_WIDE		2
#define CHASSIS_MODE_CIRCLE_LAYER			9
uint16_t Chassis_Color = UI_Color_White;

uint16_t ALIGNMENT_COLOR = UI_Color_Yellow;

uint16_t Motor_Err_Color = UI_Color_White;

Float_Data Chassis_steer_F_1,Chassis_steer_F_2,Chassis_steer_F_3,Chassis_steer_F_4;
Graph_Data G2,G3,Chassis_mode_G,Alignment_G1,Alignment_G2;

oled_task_t oled;
/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
	osDelay(OLED_TASK_START_DELAY);
	UI_select();
	memset(&Chassis_steer_F_1,0,sizeof(Chassis_steer_F_1));
	memset(&Chassis_steer_F_2,0,sizeof(Chassis_steer_F_2));
	memset(&G3,0,sizeof(G3));
	memset(&Chassis_mode_G,0,sizeof(Chassis_mode_G));
	memset(&Alignment_G1,0,sizeof(Alignment_G1));
	memset(&Alignment_G2,0,sizeof(Alignment_G2));
	
	
	Line_Draw(&Alignment_G1,ALIGNMENT_LINE_X_NAME,UI_Graph_ADD,9,ALIGNMENT_COLOR,2,ALIGNMENT_CENTER_X-ALIGNMENT_WIDE,ALIGNMENT_CENTER_Y,ALIGNMENT_CENTER_X+ALIGNMENT_WIDE,ALIGNMENT_CENTER_Y);
	UI_ReFresh(1,Alignment_G1);
	osDelay(OLED_START_TIME);
	Line_Draw(&Alignment_G2,ALIGNMENT_LINE_Y_NAME,UI_Graph_ADD,9,ALIGNMENT_COLOR,2,ALIGNMENT_CENTER_X,ALIGNMENT_CENTER_Y-ALIGNMENT_WIDE*2,ALIGNMENT_CENTER_X,ALIGNMENT_CENTER_Y);
	UI_ReFresh(1,Alignment_G2);
	osDelay(OLED_START_TIME);
	Circle_Draw(&Chassis_mode_G,CHASSIS_MODE_NAME,UI_Graph_ADD,CHASSIS_MODE_CIRCLE_LAYER,Chassis_Color,CHASSIS_MODE_CIRCLE_LINE_WIDE,chassis_mode_circle_position_x,CHASSIS_MODE_CIRCLE_POSITION_Y,CHASSIS_MODE_CIRCLE_R);
	UI_ReFresh(1,Chassis_mode_G);
	osDelay(OLED_START_TIME);
	Float_Draw(&Chassis_steer_F_1, "Chassis_steer", UI_Graph_ADD, 9, UI_Color_White, 1, 1, 80,120,900,0.0f);
	UI_ReFresh(1,Chassis_steer_F_1);
	osDelay(OLED_START_TIME);
	oled.oled_rc_ctrl = get_remote_control_point();
	oled.oled_gimbal_point = get_gimbal_move_point();
	oled.oled_chassis_point = get_chassis_move_point();
    while(1)
    {
		/****************底盘模式**********/
		if(oled.oled_chassis_point->wz_set>10000)
			chassis_mode_circle_position_x = 900;
		else if(oled.oled_chassis_point->wz_set<-10000)
			chassis_mode_circle_position_x = 1020;
		else
			chassis_mode_circle_position_x = 960;
		Circle_Draw(&Chassis_mode_G,CHASSIS_MODE_NAME,UI_Graph_Change,CHASSIS_MODE_CIRCLE_LAYER,Chassis_Color,CHASSIS_MODE_CIRCLE_LINE_WIDE,chassis_mode_circle_position_x,CHASSIS_MODE_CIRCLE_POSITION_Y,CHASSIS_MODE_CIRCLE_R);
		/****************发射*************/
		if(oled.oled_gimbal_point->shoot_switch.fric_mode == FRIC_START)
		{
			if(oled.oled_gimbal_point->shoot_switch.shoot_state == SHOOT_SINGLE_START_STATE)
				ALIGNMENT_COLOR = UI_Color_Cyan;
			else if(oled.oled_gimbal_point->shoot_switch.shoot_state == SHOOT_CONTINUOUS_START_STATE)
				ALIGNMENT_COLOR = UI_Color_Purplish_red;
			if(oled.oled_gimbal_point->shoot_switch.shoot_state == SHOOT_WITHOUT_CONTROL_STATE)
				ALIGNMENT_COLOR = UI_Color_Black;
		}
		else
			ALIGNMENT_COLOR = UI_Color_Yellow;
		Line_Draw(&Alignment_G2,ALIGNMENT_LINE_Y_NAME,UI_Graph_Change,9,ALIGNMENT_COLOR,2,ALIGNMENT_CENTER_X,ALIGNMENT_CENTER_Y-ALIGNMENT_WIDE*2,ALIGNMENT_CENTER_X,ALIGNMENT_CENTER_Y);
		/****************弹仓状态*************/
		if(oled.oled_gimbal_point->hatch_state == HATCH_OPEN)
			ALIGNMENT_COLOR = UI_Color_Black;
		else
			ALIGNMENT_COLOR = UI_Color_Yellow;
		Line_Draw(&Alignment_G1,ALIGNMENT_LINE_X_NAME,UI_Graph_Change,9,ALIGNMENT_COLOR,2,ALIGNMENT_CENTER_X-ALIGNMENT_WIDE,ALIGNMENT_CENTER_Y,ALIGNMENT_CENTER_X+ALIGNMENT_WIDE,ALIGNMENT_CENTER_Y);
		/****************底盘数据***********/
		if(oled.oled_chassis_point->chassis_state == CHASSIS_STEER_TOE)
			Motor_Err_Color = UI_Color_Orange;
		else if(oled.oled_chassis_point->chassis_state == CHASSIS_MOTOR_TOE)
			Motor_Err_Color = UI_Color_Pink;
		else if(oled.oled_chassis_point->chassis_state == CHASSIS_ALL_TOE)
			Motor_Err_Color = UI_Color_Purplish_red;
		else
			Motor_Err_Color = UI_Color_White;
		Float_Draw(&Chassis_steer_F_1, "Chassis_steer", UI_Graph_Change, 9, Motor_Err_Color, 1, 1, 80, 120,900,1.0f);
		/***************刷新******************/
		UI_ReFresh(1,Chassis_mode_G);
		osDelay(OLED_CONTROL_TIME);
		UI_ReFresh(1,Alignment_G1);
		osDelay(OLED_CONTROL_TIME);
		UI_ReFresh(1,Alignment_G2);
		osDelay(OLED_CONTROL_TIME);
		UI_ReFresh(1,Chassis_steer_F_1);
		osDelay(OLED_CONTROL_TIME);
	}
}
