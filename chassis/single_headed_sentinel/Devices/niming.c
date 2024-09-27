#include "niming.h"
#include "main.h"
#include "usart.h"

//С��ģʽ�����ֽ���ǰ�����ֽ��ں�
#define BYTE0(dwTemp) (*(char *)(&dwTemp))    //ȡ�Ͱ�λ
#define BYTE1(dwTemp) (*((char *)(&dwTemp)+1))//ȡ�߰�λ
#define BYTE2(dwTemp) (*((char *)(&dwTemp)+2))//ȡ��ʮ��λ
#define BYTE3(dwTemp) (*((char *)(&dwTemp)+3))//ȡ�߶�ʮ��λ

/**
  * @brief  ������λ��Э��
  */
void sent_data1(float A,float B,int16_t C)
//					����һ��2�ֽ�����A�����ֽ�����B��4�ֽ�����C
{	
		uint8_t BUFF[100];
		uint8_t _cnt = 0;
		uint16_t flen;
	
		BUFF[_cnt++]=0xAB;//��ͷ
		BUFF[_cnt++]=0xFF;//Դ��ַ
		BUFF[_cnt++]=0xFF;//Ŀ���ַ
		BUFF[_cnt++]=0xF1;//������(ID)
		BUFF[_cnt++]=10;//���ݳ���(2�ֽ�)
		BUFF[_cnt++]=0;//���ݳ���
		BUFF[_cnt++]=BYTE0(A);//��������A��2�ֽڣ�
		BUFF[_cnt++]=BYTE1(A);//��������A
		BUFF[_cnt++]=BYTE2(A);//��������A��2�ֽڣ�
		BUFF[_cnt++]=BYTE3(A);//��������A
	
		BUFF[_cnt++]=BYTE0(B);//��������B��2�ֽڣ�
		BUFF[_cnt++]=BYTE1(B);//��������B
		BUFF[_cnt++]=BYTE2(B);//��������A��2�ֽڣ�
		BUFF[_cnt++]=BYTE3(B);//��������A
	
		BUFF[_cnt++]=BYTE0(C);
		BUFF[_cnt++]=BYTE1(C);
	
		uint8_t sumcheck = 0;
		uint8_t addcheck = 0;
		flen = BUFF[4] + BUFF[5] * 255;
		for(uint16_t i=0; i < (flen+6); i++)//У�����
		{
			sumcheck += BUFF[i]; 
			addcheck += sumcheck; 
		}
		
		BUFF[_cnt++] = sumcheck;//��У��
		BUFF[_cnt++] = addcheck;//����У��
		HAL_UART_Transmit(&huart7,BUFF,_cnt,0xffff);//���ڷ���
}
