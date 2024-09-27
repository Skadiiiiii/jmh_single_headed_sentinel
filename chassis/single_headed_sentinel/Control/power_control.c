#include "power_control.h"
#include "M3508_motor.h"
#include "chassis_control.h"
#include "pid.h"
#include "bsp_can.h"
//positionpid_t chassis_powerPID = PowerLimitPIDInit;
//#undef chassis_powerPID
Chassis_PowerLimit_t Chassis_PowerLimit;	
	
PowerMeter_t Power_Meter;
PowerMeter_Rec_t PowerMeter_Rec;

float coe[4];
void chassis_power_control(Chassis_PowerLimit_t *chassis_power_control)
{
	Chassis_PowerLimit.SumCurrent_IN = fabs((float)M3508s_chassis[0].set_current) + 
										fabs((float)M3508s_chassis[1].set_current) +
										fabs((float)M3508s_chassis[2].set_current) + 
											fabs((float)M3508s_chassis[3].set_current);
	for(int i = 0; i < 4; i++)
	{
		coe[i] = ((float)(M3508s_chassis[i].set_current)) / ((float)(Chassis_PowerLimit.SumCurrent_IN));
	}
	//--偷缓冲功率20w（60-20）
	if(Power_Meter.Power < 60)
	{
//		Chassis_PowerLimit.powerBuffRatio = (float)chassis_power_buffer / 60.0f / chassis_powerPID.pwm; 
		Chassis_PowerLimit.powerBuffRatio = (float)Power_Meter.Power / 30.0f; 
		Chassis_PowerLimit.SumCurrent_Out = Chassis_PowerLimit.SumCurrent_IN * Chassis_PowerLimit.powerBuffRatio;
	}
	else if (Power_Meter.Power >= 60)
	{
		Chassis_PowerLimit.SumCurrent_Out = Chassis_PowerLimit.SumCurrent_IN;
	}
	
//按照百分比分配最大电流
	for (int i = 0; i < 4; i++)
	{
		if(isnan(Chassis_PowerLimit.SumCurrent_Out * coe[i])!=1)
		{
			M3508s_chassis[i].set_current = (float)(Chassis_PowerLimit.SumCurrent_Out * coe[i]);
		}
	}
}

void PowerMeter_Update(Can_Export_Data_t RxMessage)
{
  memcpy(&PowerMeter_Rec.data, RxMessage.CAN_RxMessage, 8);
	Power_Meter.Power = PowerMeter_Rec.Pack.Shunt_Current * PowerMeter_Rec.Pack.voltageVal /1000/1000;
//	Power_Meter.infoUpdateFrame++;
}

