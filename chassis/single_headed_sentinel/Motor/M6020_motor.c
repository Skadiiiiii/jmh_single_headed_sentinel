#include "M6020_motor.h"
#include <stdio.h>
#include "bsp_can.h"

M6020s_t M6020s_Yaw;

/**
  * @brief  获取云台Yaw轴6020的数据
  */
void M6020_Yaw_getInfo(Can_Export_Data_t RxMessage)
{
    //解包数据，数据格式详见C620电调说明书P33
    M6020s_Yaw.last_rotor_angle = M6020s_Yaw.rotor_angle;
    M6020s_Yaw.rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M6020s_Yaw.rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M6020s_Yaw.torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M6020s_Yaw.temp = RxMessage.CAN_RxMessage[6];

//    if (M6020s_Yaw.rotor_angle - M6020s_Yaw.last_rotor_angle < -4096)
//    {
//        M6020s_Yaw.turn_count++;
//    }

//    if (M6020s_Yaw.last_rotor_angle - M6020s_Yaw.rotor_angle < -4096)
//    {
//        M6020s_Yaw.turn_count--;
//    }

////    M6020s_Yaw.total_angle = M6020s_Yaw.rotor_angle + (8192 * M6020s_Yaw.turn_count);
//			M6020s_Yaw.target_total_rotor_angle = M6020s_Yaw.target_rotor_angle + (8192 * M6020s_Yaw.turn_count);

    //帧率统计，数据更新标志位
    M6020s_Yaw.InfoUpdateFrame++;
    M6020s_Yaw.InfoUpdateFlag = 1;
}

