#include "string.h"
#include "stdlib.h"
#include "dr16.h"
#include "usart.h"
#include "main.h"
#include "M6020_motor.h"

/*******************************用户数据定义************************************/
ControlSwitch_t ControlSwitch;
DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
/*******************************************************************************/


/**
  * @brief  获取A板发送的dr16数据
  */
void dr16_getInfo(Can_Export_Data_t DR16_Receive_Data)
{   
    //解包数据，数据格式详见C620电调说明书P33
    DR16_Export_Data.Robot_TargetValue.Omega_Value = (int16_t)(DR16_Receive_Data.CAN_RxMessage[0] << 8 | DR16_Receive_Data.CAN_RxMessage[1]);
    DR16_Export_Data.Robot_TargetValue.Pitch_Value = (int16_t)(DR16_Receive_Data.CAN_RxMessage[2] << 8 | DR16_Receive_Data.CAN_RxMessage[3]);
    DR16_Export_Data.ChassisWorkMode = (int16_t)(DR16_Receive_Data.CAN_RxMessage[6] << 8 | DR16_Receive_Data.CAN_RxMessage[7]);
}


