#include "main.h"
#include "chassis_control.h"
#include "cloud_control.h"
#include "cmsis_os.h"
#include "Robot_control_task.h"

//--离线状态
//Offline_Estimate_Typedef robot_offline_estimate;


///**
//  * @brief          返回离线情况评估
//**/
//uint8_t Get_Robot_OfflineState(void)
//{
//    return robot_offline_estimate.offline_state;
//}

/**
  * @brief    机器人主控制任务
**/
void Robot_Control(void const *argument)
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	for (;;)
  {
		Robot_Control_Fun();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

