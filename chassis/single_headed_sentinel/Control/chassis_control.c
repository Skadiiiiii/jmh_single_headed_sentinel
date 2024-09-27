#include "chassis_control.h"
#include "dr16.h"
#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "M2006_motor.h"
#include "pid.h"
#include "math.h"
#include "power_control.h"

int16_t speed_buff[4];

/**
  * @brief  取变量的绝对值
  */
float abs(float num)
{
		int temp;
		if(num<0) temp=-num;
		else temp=num;
		return temp;
}

/**
  * @brief  麦轮解算
  */
static void MecanumCalculate(float X_Move,float Y_Move,float Yaw ,int16_t *Speed)
{
		float target_speed[4];
    float MaxSpeed = 0.0f;
    float Param = 1.0f;
 
	  target_speed[0] = 	X_Move - Y_Move + Yaw;
		target_speed[1] =    X_Move + Y_Move + Yaw;
		target_speed[2] =   Yaw - X_Move + Y_Move  ;
		target_speed[3]=    Yaw -X_Move - Y_Move  ;	

    for(uint8_t i = 0 ; i<4 ; i ++)
    {
        if(abs(target_speed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(target_speed[i]);
        }
    }

    if (MaxSpeed > 10000)
    {
        Param = (float)10000 / MaxSpeed;
    }

    Speed[0] = target_speed[0] * Param;
    Speed[1] = target_speed[1] * Param;
    Speed[2] = target_speed[2] * Param;
    Speed[3] = target_speed[3] * Param;
}

/**
 * @brief 全向公式
 */
static float* Omnidirectional_Formula(float Vx, float Vy)
{
    float RadRaw = 0.0f;
		static float Chassis[2];
	
    float angle = (M6020s_Yaw.rotor_angle - M6020s_Yaw_Angle_Centre) / M6020_mAngleRatio; //机械角度偏差
    RadRaw = angle * DEG_TO_RAD;                           //弧度偏差
    //全向移动公式。
    Chassis[0] = Vx * cos(RadRaw) - Vy * sin(RadRaw);
    Chassis[1] = Vy * cos(RadRaw) + Vx * sin(RadRaw);
		return Chassis;
}

/**
  * @brief  底盘使能
  */
static void Ship_ChassisWorkMode(float Vx, float Vy,float VOmega)
{

	
	MecanumCalculate(Vx,Vy,VOmega,speed_buff);		
		
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
	}

	set_M3508_chassis_voltage(
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current, 
							M3508s_chassis[3].set_current);		
}

/**
  * @brief  底盘跟随模式
  */
static void Ship_ChassisWorkMode_follow(float Vx, float Vy)
{	
	int16_t Chassis_target_rotor_speed;

	Chassis_target_rotor_speed = pid_calc_cloud(&motor_pid_chassis_pos,M6020s_Yaw_Angle_Centre,M6020s_Yaw.rotor_angle);
	 
	MecanumCalculate(Vx,Vy,Chassis_target_rotor_speed,speed_buff);	
			
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
	}
	
//	chassis_power_control(&Chassis_PowerLimit);

	set_M3508_chassis_voltage(
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current, 
							M3508s_chassis[3].set_current);			
}

/**
  * @brief  底盘小陀螺模式
  */
static void Ship_ChassisWorkMode_Tuoluo(float Vx, float Vy)
{
	
	float* Chassis = Omnidirectional_Formula(Vx,Vy);
	float target_Vomega = 2000;
	
	MecanumCalculate(Chassis[0],Chassis[1],target_Vomega,speed_buff);
								
	for (uint8_t i = 0; i < 4; i++) 
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
	}
	
	chassis_power_control(&Chassis_PowerLimit);

	set_M3508_chassis_voltage(
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current, 
							M3508s_chassis[3].set_current);
}

/**
  * @brief  拨盘使能
  */
static void Ship_ChassisWorkMode_shoot(float speed)
{
	M2006s.set_current = pid_calc(&motor_pid_shoot,speed,M2006s.rotor_speed);
	
	set_M2006_voltage(
							0, 
							M2006s.set_current, 
							0, 
							0);
}

/**
  * @brief  底盘失能
  */
static void Robot_control_chassis_disable()
{
	for(uint8_t i = 0;i < 4;i++)
	{
		M3508s_chassis[i].set_current = 0;
	}
	set_M3508_chassis_voltage(0,0,0,0);
}

/**
  * @brief  机器人主控制
  */
static void Robot_control ()        
{
	if(rc.sw1 == 3  && rc.sw2 == 2)//左中右下（底盘）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Chassis;
		Ship_ChassisWorkMode(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,3.0f*DR16_Export_Data.Robot_TargetValue.Omega_Value);
	}
	else if(rc.sw1 == 2  && rc.sw2 == 3)//左下右中（云台）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Cloud;
		Robot_control_chassis_disable(); 
	}
	else if(rc.sw1 == 3  && rc.sw2 == 3)//左中右中（跟随）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Follow;
		Ship_ChassisWorkMode_follow(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	
	else if(rc.sw1 == 3  && rc.sw2 == 1)//左中右上（小陀螺）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Tuoluo;
		Ship_ChassisWorkMode_Tuoluo(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(rc.sw1 == 1  && rc.sw2 == 3)//左上右中（发射）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Shoot;
		Ship_ChassisWorkMode_shoot(7.0f*DR16_Export_Data.Robot_TargetValue.Pitch_Value);
		Ship_ChassisWorkMode_follow(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else
	{
		Robot_control_chassis_disable();	
		DR16_Export_Data.ChassisWorkMode = 0;
	}
}

/**
  * @brief    遥控器控制机器人
**/
void Robot_Control_Fun()
{
	RemoteControl_Output();
	Robot_control();
}

/**
  * @brief    遥控器控制机器人
**/
void Robot_Control_Disable()
{
	Robot_control_chassis_disable();	
	DR16_Export_Data.ChassisWorkMode = 0;
}

