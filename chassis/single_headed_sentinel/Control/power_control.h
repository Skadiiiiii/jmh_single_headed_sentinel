#ifndef __DRIVER_CLASSIS_PowerLimit_H
#define __DRIVER_CLASSIS_PowerLimit_H

#include <stdint.h>
#include "string.h"
#include "math.h"
#include "M3508_motor.h"
#define MomentCurrent 0.01562f//KT(力矩参数) * I(力矩电流) = torque(力矩)
#define CurrentConstant (float) (0.0012207f)//I = torque_current(A) = CurrentConstant * set_current
//Mechanical_power(机械功率) = torque(力矩) * rotor_speed(转速) / 9.55f
//Power(电机功率) = Mechanical_power + k1 * rotor_speed^2 + k2 * torque^2 + a(静态电流)
#define ConstantMoment 1.196e-6
typedef struct
{
	int32_t SumCurrent_IN;//输入的电流总和
	int32_t SumCurrent_Out;//最后计算电流总和
	float powerBuffRatio;
	
	float LimitPowerMax; //当前限制的功率大小 单位W    
	float chassis_powerBuff;
	float RealChassisPower;
	M3508s_t  motor[4]; 

}Chassis_PowerLimit_t;

#define PowerLimitPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            1.72f,           \
            0.0f,           \
            0.0f,          \
						0.0f,						\
            0,              \
            0,              \
            0,              \
            100,            \
            700,              \
            15000,              \
            &Position_PID, \
    }
		
#define SupercellPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            5.10f,           \
            0.0f,           \
            0.0f,          \
						0.0f,						\
            0,              \
            0,              \
            0,              \
            100,            \
            0,              \
            0,              \
            &Position_PID, \
    }
		
typedef union 
{
	struct
    {
        float voltageVal;//mV
        float Shunt_Current;//mA
    }Pack;
		uint8_t data[8];
		
}PowerMeter_Rec_t;
		
		
typedef struct 
{
		float infoUpdateFrame;
		float OffLineFlag;
		float Power;
	
}PowerMeter_t;



void chassis_power_control(Chassis_PowerLimit_t *chassis_power_control);
void PowerMeter_Update(Can_Export_Data_t RxMessage);
extern Chassis_PowerLimit_t Chassis_PowerLimit;
#endif


