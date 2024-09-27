#include "cmsis_os.h"
#include "bsp_can.h"
#include "dr16.h"
#include "M6020_motor.h"

static void Can_Send_Fun()
{
	DR16_0x175_Can1_SendData(DR16_Export_Data.Robot_TargetValue.Omega_Value,DR16_Export_Data.Robot_TargetValue.Pitch_Value,0,DR16_Export_Data.ChassisWorkMode);
//	M6020_Yaw_0x185_Can1_SendData(M6020s_Yaw.rotor_angle,M6020s_Yaw.rotor_speed,0,0);
}




/**
  * @brief    can板间通讯函数
**/
void CAN_Send(void const *argument)
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	for (;;)
  {
		Can_Send_Fun();
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}


