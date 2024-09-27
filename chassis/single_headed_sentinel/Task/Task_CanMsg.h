#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H
#include "bsp_can.h"

#define M6020_Yaw_ID 0x205
#define M3508_Chassis_Begin_ID 0x201
#define M3508_Chassis_End_ID 0x204
#define M2006_Shoot_ID 0x206

extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;

#endif 

