#include "main.h"
#include "chassis_control.h"
#include "cloud_control.h"
#include "cmsis_os.h"
#include "Robot_control_task.h"

//--����״̬
//Offline_Estimate_Typedef robot_offline_estimate;


///**
//  * @brief          ���������������
//**/
//uint8_t Get_Robot_OfflineState(void)
//{
//    return robot_offline_estimate.offline_state;
//}

/**
  * @brief    ����������������
**/
void Robot_Control(void const *argument)
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ����ܿ���
	for (;;)
  {
		Robot_Control_Fun();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

