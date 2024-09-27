#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H
#include "bsp_can.h"

#define M6020_Yaw_ID 0x205
#define M6020_Pitch_ID 0x206
#define M3508_Shoot_Begin_ID 0x207
#define M3508_Shoot_End_ID 0x208
#define DR16_C_ID 0x175

extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;

#endif 

