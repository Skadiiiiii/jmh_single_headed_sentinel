#include "string.h"
#include "stdlib.h"
#include "dr16.h"
#include "usart.h"
#include "main.h"
#include "M6020_motor.h"

/*******************************�û����ݶ���************************************/
ControlSwitch_t ControlSwitch;
DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
/*******************************************************************************/


/**
  * @brief  ��ȡA�巢�͵�dr16����
  */
void dr16_getInfo(Can_Export_Data_t DR16_Receive_Data)
{   
    //������ݣ����ݸ�ʽ���C620���˵����P33
    DR16_Export_Data.Robot_TargetValue.Omega_Value = (int16_t)(DR16_Receive_Data.CAN_RxMessage[0] << 8 | DR16_Receive_Data.CAN_RxMessage[1]);
    DR16_Export_Data.Robot_TargetValue.Pitch_Value = (int16_t)(DR16_Receive_Data.CAN_RxMessage[2] << 8 | DR16_Receive_Data.CAN_RxMessage[3]);
    DR16_Export_Data.ChassisWorkMode = (int16_t)(DR16_Receive_Data.CAN_RxMessage[6] << 8 | DR16_Receive_Data.CAN_RxMessage[7]);
}


