#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define M6020s_Yaw_Angle_Centre 1450
#define M6020s_Pitch_Angle_Centre 1410
//#define M6020s_Pitch_Angle_Max 2000
//#define M6020s_Pitch_Angle_Min 1200

#define M6020s_Pitch_Angle_Max 4500
#define M6020s_Pitch_Angle_Min 3750

typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
		float    target_rotor_angle;
    int16_t  rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
		uint16_t last_rotor_angle;
		int32_t  turn_count;
		float 	 total_angle;
		float 	 target_total_rotor_angle;
	
	  uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
} M6020s_t;

extern M6020s_t M6020s_Yaw;
extern M6020s_t M6020s_Pitch;

void M6020_TargetAngle_init(void);
void M6020_Yaw_getInfo(Can_Export_Data_t RxMessage);
void M6020_Pitch_getInfo(Can_Export_Data_t RxMessage);
void set_M6020_yaw_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);//形参表示4个电机的电压值
void set_M6020_pitch_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif
