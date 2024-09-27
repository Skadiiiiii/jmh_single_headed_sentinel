#include "Task_CanMsg.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "M2006_Motor.h"
#include "dr16.h"

/**
  * @brief  can1��������
  */
void CAN1_REC(void const *argument)
{
	Can_Export_Data_t Can_Export_Data;
	uint32_t ID;
	for (;;)
	{
		xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
		ID = Can_Export_Data.CAN_RxHeader.StdId;
		if (ID == M6020_Yaw_ID)
		{
			M6020_Yaw_getInfo(Can_Export_Data);
		}
		else if (ID == DR16_C_ID)
		{
			dr16_getInfo(Can_Export_Data);//dr16
		}
	}
}

/**
  * @brief  can2��������
  */
void CAN2_REC(void const *argument)
{
	uint32_t ID;
	Can_Export_Data_t Can_Export_Data;
	for (;;)
	{
		xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
		ID = Can_Export_Data.CAN_RxHeader.StdId;
		if (ID == M6020_Pitch_ID)
		{
			M6020_Pitch_getInfo(Can_Export_Data);//Pitch
		}
		else if (ID >= M3508_Shoot_Begin_ID || ID <= M3508_Shoot_End_ID)
		{
			M3508_shoot_getInfo(Can_Export_Data);//Shoot
		}

	}
}

