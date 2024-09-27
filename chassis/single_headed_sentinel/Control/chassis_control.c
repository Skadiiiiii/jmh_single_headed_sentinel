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
  * @brief  ȡ�����ľ���ֵ
  */
float abs(float num)
{
		int temp;
		if(num<0) temp=-num;
		else temp=num;
		return temp;
}

/**
  * @brief  ���ֽ���
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
 * @brief ȫ��ʽ
 */
static float* Omnidirectional_Formula(float Vx, float Vy)
{
    float RadRaw = 0.0f;
		static float Chassis[2];
	
    float angle = (M6020s_Yaw.rotor_angle - M6020s_Yaw_Angle_Centre) / M6020_mAngleRatio; //��е�Ƕ�ƫ��
    RadRaw = angle * DEG_TO_RAD;                           //����ƫ��
    //ȫ���ƶ���ʽ��
    Chassis[0] = Vx * cos(RadRaw) - Vy * sin(RadRaw);
    Chassis[1] = Vy * cos(RadRaw) + Vx * sin(RadRaw);
		return Chassis;
}

/**
  * @brief  ����ʹ��
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
  * @brief  ���̸���ģʽ
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
  * @brief  ����С����ģʽ
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
  * @brief  ����ʹ��
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
  * @brief  ����ʧ��
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
  * @brief  ������������
  */
static void Robot_control ()        
{
	if(rc.sw1 == 3  && rc.sw2 == 2)//�������£����̣�
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Chassis;
		Ship_ChassisWorkMode(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,3.0f*DR16_Export_Data.Robot_TargetValue.Omega_Value);
	}
	else if(rc.sw1 == 2  && rc.sw2 == 3)//�������У���̨��
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Cloud;
		Robot_control_chassis_disable(); 
	}
	else if(rc.sw1 == 3  && rc.sw2 == 3)//�������У����棩
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Follow;
		Ship_ChassisWorkMode_follow(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	
	else if(rc.sw1 == 3  && rc.sw2 == 1)//�������ϣ�С���ݣ�
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Tuoluo;
		Ship_ChassisWorkMode_Tuoluo(7.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,7.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(rc.sw1 == 1  && rc.sw2 == 3)//�������У����䣩
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
  * @brief    ң�������ƻ�����
**/
void Robot_Control_Fun()
{
	RemoteControl_Output();
	Robot_control();
}

/**
  * @brief    ң�������ƻ�����
**/
void Robot_Control_Disable()
{
	Robot_control_chassis_disable();	
	DR16_Export_Data.ChassisWorkMode = 0;
}

