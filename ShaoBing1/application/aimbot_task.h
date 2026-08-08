#ifndef _AIMBOT_H_
#define _AIMBOT_H_

#include "main.h"
#include "struct_typedef.h"


#define AIMBOT_CONTROL_TIME 100

#define AIMBOT_AUTO_FIRE_THRE	20.0f


typedef enum
{
	AIMBOT_OFF = 0,
	AIMBOT_ON = 1,
	AIMBOT_RESET = 2,		//reset_tracker
	
	AIMBOT_GET = 3,
	AIMBOT_OUT = 4,
}aimbot_mode_e;

typedef enum
{
	AIMBOT_FIRE_ON = 1,
	AIMBOT_FIRE_OFF = 0,
}aimbot_auto_fire_e;

typedef __packed struct
{
	uint8_t header;
	bool detect_color : 1; //0-red 1-blue
	uint8_t reset_tracker : 1;
	uint8_t reserved : 6;
	
	float roll;
	float pitch;
	float yaw;
	float aim_x;
	float aim_y;
	float aim_z;
	
	uint16_t checksum;
}Vision_Tx_Data_t;

typedef __packed struct
{
	uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;             // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;		// 2-balance 3-outpost 4-normal
	uint8_t reserved : 1;
	
	float x;
	float y;
	float z;
	float yaw;
	float vx;
	float vy;
	float vz;
	float v_yaw;
	float r1;
	float r2;
	float dz;
	uint16_t checksum;
}Vision_Rx_Data_t;

typedef struct
{
	fp32 t0;
	fp32 t1;
	
	fp32 ft1;
	fp32 ft2;
	
	fp32 t_out;			//牛顿迭代算出来的飞行时间
	
}aimbot_count_t;

typedef struct
{
  const fp32 *aimbot_INT_angle_point;
  const fp32 *aimbot_INT_gyro_point;
  const fp32 *aimbot_INT_quat_point;
  
	
	uint8_t aimbot_mode;
	uint8_t aimbot_state;
	uint8_t aimbot_auto_fire_state;
	
	Vision_Rx_Data_t Vision_Rx_Data;
	Vision_Tx_Data_t Vision_Tx_Data;
	
	aimbot_count_t	aimbot_count;		//算出来的飞行时间
	
	fp32 target_x;
	fp32 target_y;
	fp32 temple_z;
	fp32 target_z;
	
	fp32 last_x;
	fp32 last_y;
	fp32 vx;
	fp32 vy;
	
  fp32 target_yaw;
  fp32 target_pitch;
  fp32 target_roll;
  
}aimbot_task_t;

extern void aimbot_receive_program(void);
extern void aimbot_mode_set(uint8_t tracker_state);
void aimbot_flight_time_count(aimbot_count_t *aimbot_count);
uint8_t aimbot_auto_fire_control(fp32 yaw, fp32 target_yaw);
extern void aimbot_task(void const *pvParameters);
extern void aimbot_init(aimbot_task_t *aimbot_task_init);
extern void aimbot_feedback_update(aimbot_task_t *aimbot_feedback);
extern void aimbot_control_loop(aimbot_task_t *aimbot_control);
extern const aimbot_task_t *get_aimbot_point(void);

#endif
