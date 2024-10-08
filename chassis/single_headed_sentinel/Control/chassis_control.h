#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "main.h"

#define WorkMode_Cloud 12
#define WorkMode_Chassis 21
#define WorkMode_Follow 22
#define WorkMode_Tuoluo 23
#define WorkMode_Shoot 32
#define WorkMode_Disable 11
#define M6020s_Yaw_Angle_Centre 1450

#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define DEG_TO_RAD 0.017453292519943295769236907684886f

float abs(float num);
void Robot_Control_Disable(void);
void Robot_Control_Fun(void);

#endif
