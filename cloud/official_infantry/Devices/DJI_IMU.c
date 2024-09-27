#include "DJI_IMU.h"
#include "INS_task.h"
//����can������֡�����ֽ���Ϊ8
//ŷ���ǵ�һ��������ݾ���һ��float���ͱ�������4���ֽ�
//��Ҳ�����ֻ�ܷ�������
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
IMU_Rec_Data_t DJI_C_IMU;
//���ձ���
IMU_CAL_t IMU_CAL;

extern float INS_angle[3];      //euler angle, unit rad.ŷ���� ��λ rad
extern float INS_gyro[3];

/*��һ�ַ��ͷ�ʽ��������*/
//ŷ����
void Euler_Send_Fun()
{
	DJI_C_IMU.yaw = INS_angle[0] * (180/PI) + 180.0f;
	DJI_C_IMU.pitch = -INS_angle[2] * (180/PI) + 180.0f;
	DJI_C_IMU.Gyro_z = INS_gyro[2] * (180/PI);
	DJI_C_IMU.Gyro_y = INS_gyro[0] * (180/PI);
	
	Euler_Send.yaw = INS_angle[0];
	Gyro_Send.Gyro_y = INS_gyro[0];

	//yaw��Ĺ��㴦��
  if (DJI_C_IMU.yaw - DJI_C_IMU.last_yaw < -300.0f)
	{
			DJI_C_IMU.turnCounts++;
	}
	if (DJI_C_IMU.last_yaw - DJI_C_IMU.yaw < -300.0f)
	{
			DJI_C_IMU.turnCounts--;
	}
	
	DJI_C_IMU.total_yaw = DJI_C_IMU.yaw + DJI_C_IMU.turnCounts * 360.0f;

	DJI_C_IMU.last_yaw = DJI_C_IMU.yaw;
	
//	//pitch��Ĺ��㴦��
//	if (DJI_C_IMU.pitch - DJI_C_IMU.last_pitch < -300.0f)
//	{
//			DJI_C_IMU.pitch_turnCounts++;
//	}
//	if (DJI_C_IMU.last_pitch - DJI_C_IMU.pitch < -300.0f)
//	{
//			DJI_C_IMU.pitch_turnCounts--;
//	}
//	
//	DJI_C_IMU.total_pitch = DJI_C_IMU.pitch + DJI_C_IMU.pitch_turnCounts * 360.0f;

//	DJI_C_IMU.last_pitch = DJI_C_IMU.pitch;
}

/**
  * @brief  ��A�巢������������
  */
void IMU_0x185_Can1_SendData(Euler_Send_u Euler_Send,Gyro_Send_u Gyro_Send)
{	
	CAN_TxHeaderTypeDef TxMessage;
	uint8_t DATA[8];
		
  TxMessage.IDE = CAN_ID_STD;     //����ID����
	TxMessage.RTR = CAN_RTR_DATA;   //���ô�������֡
	TxMessage.DLC = 0x08;              //�������ݳ���
	TxMessage.StdId = 0x185;        //����ID��
	
	DATA[0] = Euler_Send.Euler_Angle[0];
	DATA[1] = Euler_Send.Euler_Angle[1];
	DATA[2] = Euler_Send.Euler_Angle[2];
	DATA[3] = Euler_Send.Euler_Angle[3];
	DATA[4] = Gyro_Send.Gyro_zy[0];
	DATA[5] = Gyro_Send.Gyro_zy[1];
	DATA[6] = Gyro_Send.Gyro_zy[2];
	DATA[7] = Gyro_Send.Gyro_zy[3];
		
  HAL_CAN_AddTxMessage(&hcan1,&TxMessage,DATA,0);
}
