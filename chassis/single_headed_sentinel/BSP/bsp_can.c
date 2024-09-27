#include "bsp_can.h"
#include "DJI_IMU.h"
#include "M6020_motor.h"
#include "M2006_motor.h"
#include "M3508_motor.h"

Can_Data_t Can_Data[2] = Can_DataGroundInit;
Can_Export_Data_t Can1_Export_Data;//�����洢���յ���CAN1����
Can_Export_Data_t Can2_Export_Data;//�����洢���յ���CAN2����

/**
  * @brief  CAN1��������ʼ��
  */
void CAN_1_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 0;                       //CAN��������ţ���Χ0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
  sFilterConfig.FilterIdHigh = 0x0000;			//32λ�£��洢Ҫ����ID�ĸ�16λ
  sFilterConfig.FilterIdLow = 0x0000;					//32λ�£��洢Ҫ����ID�ĵ�16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//����ģʽ�£��洢��������
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//���������
  sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1) ;
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  CAN2��������ʼ��
  */
 void CAN_2_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 14;                       //CAN��������ţ���Χ0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN������ģʽ������ģʽ���б�ģʽ
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN�������߶ȣ�16λ��32λ
  sFilterConfig.FilterIdHigh = 0x0000;			//32λ�£��洢Ҫ����ID�ĸ�16λ
  sFilterConfig.FilterIdLow = 0x0000;					//32λ�£��洢Ҫ����ID�ĵ�16λ
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//����ģʽ�£��洢��������
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//���������
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_CAN_Start(&hcan2) ;
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  ����CAN���յ�������
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN���߽��ջص�����
{
	if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can1_Export_Data.CAN_RxHeader, Can1_Export_Data.CAN_RxMessage); //��CAN���߽�������
		
		xQueueSendToBackFromISR(CAN1_ReceiveHandle, &Can1_Export_Data, 0);//�����ݷ��͵�FreeRTOS��Ϣ����
  }
	
  else if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can2_Export_Data.CAN_RxHeader, Can2_Export_Data.CAN_RxMessage); //��CAN���߽�������
		
		xQueueSendToBackFromISR(CAN2_ReceiveHandle, &Can2_Export_Data, 0);//�����ݷ��͵�FreeRTOS��Ϣ����
  }
}
                                                                                                    
/**
  * @brief  ����CAN��Ϣ���������
  */
//void CAN_RxMessage_Export_Data(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type)
//{
//	Can_Export_Data_t Can_Export_Data[2];//���ڴ洢���յ���can����
//	uint8_t Canx_type = Can_type - 1;
//	HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0,
//											 &Can_Data[Canx_type].CAN_RxTypedef.CANx_RxHeader,
//											 Can_Data[Canx_type].CAN_RxTypedef.CANx_RxMessage);//��can���߽������ݣ����洢��Can_Data��

//	Can_Export_Data[Canx_type].CAN_RxHeader = Can_Data[Canx_type].CAN_RxTypedef.CANx_RxHeader;
//	memcpy(&Can_Export_Data[Canx_type].CAN_RxMessage,
//				 Can_Data[Canx_type].CAN_RxTypedef.CANx_RxMessage,
//				 sizeof(uint8_t[8]));//�����յ���can���ݸ��Ƶ�Can_Export_Data��

//	xQueueSendToBackFromISR(CANx_Handle, &Can_Export_Data[Canx_type], 0);//�����ݷ��͵�FreeRTOS��Ϣ����
//}



	
