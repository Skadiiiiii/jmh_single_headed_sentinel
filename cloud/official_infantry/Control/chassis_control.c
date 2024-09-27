//#include "chassis_control.h"
//#include "dr16.h"
//#include "bsp_can.h"
//#include "M3508_motor.h"
//#include "M6020_motor.h"
//#include "pid.h"
//#include "math.h"
//#include "DJI_IMU.h"
//#include "iir_filter.h"

//uint8_t imu_start_flag;

///**
// * @brief ��ȡ����������������ݣ�����Ϊ��׼
// */
//static void read_start_imu(void)
//{
//	uint16_t imu_start_angle;
//	if(imu_start_flag == 0)
//	{
//		for(uint8_t i = 0;i < 10;i++)
//		{
//			M6020s_Yaw.target_rotor_angle = imu_start_angle;
//			imu_start_angle = 22.7527f * DJI_C_IMU.total_yaw - 2646;
//			imu_start_flag = 1;
//			DR16_Export_Data.Robot_TargetValue.Omega_Value = 0;
//		}
//	}
//}

///**
//  * @brief  ��̨ʹ��
//  */
//static void Ship_ChassisWorkMode_cloud(float delta_yaw,float delta_pitch)
//{
//	read_start_imu();

//	M6020s_Yaw.target_rotor_angle   += delta_yaw;
//	M6020s_Pitch.target_rotor_angle += delta_pitch;
//	
//	if(M6020s_Pitch.target_rotor_angle < 1200)
//	{
//		M6020s_Pitch.target_rotor_angle = 1200;
//	}
//	else if(M6020s_Pitch.target_rotor_angle > 2000)
//	{
//		M6020s_Pitch.target_rotor_angle = 2000;
//	}

//	M6020s_Yaw.set_voltage   = pid_CascadeCalc_yaw(&motor_pid_Cas_Yaw, M6020s_Yaw.target_rotor_angle,22.7527f * DJI_C_IMU.total_yaw - 2646,M6020s_Yaw.rotor_speed);
//	
//  M6020s_Pitch.set_voltage = pid_CascadeCalc_pitch(&motor_pid_Cas_Pitch, M6020s_Pitch.target_rotor_angle,M6020s_Pitch.rotor_angle,M6020s_Pitch.rotor_speed);
//	
//	set_M6020_yaw_voltage(
//							M6020s_Yaw.set_voltage, //yaw
//							0, //pitch
//							0, 
//							0);
//	set_M6020_pitch_voltage(
//							0, //yaw
//							M6020s_Pitch.set_voltage, //pitch
//							0, 
//							0);
//}


///**
//  * @brief  ��̨ʧ��
//  */
//static void Robot_control_cloud_disable()
//{

//	M6020s_Yaw.set_voltage = 0; 
//	M6020s_Pitch.set_voltage = 0;

//	set_M6020_yaw_voltage(0,0,0,0);
//	set_M6020_pitch_voltage(0,0,0,0);
//}

///**
//  * @brief  ������������
//  */
//void Robot_control (void)        
//{
//	if(DR16_Export_Data.ChassisWorkMode == 1 || DR16_Export_Data.ChassisWorkMode == 2)
//	{
//		Ship_ChassisWorkMode_cloud(-0.02f*DR16_Export_Data.Robot_TargetValue.Omega_Value,0.02f*DR16_Export_Data.Robot_TargetValue.Pitch_Value);
//	}
//	else
//	{
//		Robot_control_cloud_disable();	
//		imu_start_flag = 0;
//		M6020s_Pitch.target_rotor_angle = 1410;
//		
//		yaw_speed_filter[0] = 0.0;
//		yaw_speed_filter[1] = 0.0;
//		yaw_speed_filter[2] = 0.0;
//		yaw_target_filter[0] = 0.0;
//		yaw_target_filter[1] = 0.0;
//		yaw_target_filter[2] = 0.0;
//		
////		pitch_speed_filter[0] = 0.0;
////		pitch_speed_filter[1] = 0.0;
////		pitch_speed_filter[2] = 0.0;
////		pitch_speed_filter [0] = 0.0;
////		pitch_speed_filter [1] = 0.0;
////		pitch_speed_filter [2] = 0.0;
//	}

//}

