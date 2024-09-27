#include "Task_Check.h" 
#include "dr16.h"

void DEV_Check(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //ÿ200����ǿ�ƽ����ܿ���

	for(;;)
	{
		Check_DR16();                     //DR16���

		if(rc.OffLineFlag == 1)
		{
			DR16_Export_Data.Alldead = 1;
		}
		else
		{
			DR16_Export_Data.Alldead = 0;
		}

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

