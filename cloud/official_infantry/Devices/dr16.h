 #ifndef __DR16_REMOTE__H__
#define __DR16_REMOTE__H__

#include "usart.h"
#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3

typedef enum
{
    RemotePole_UP = 1,  //��
    RemotePole_MID = 3, //��
    RemotePole_DOWM = 2 //��
} RemotePole_e;

typedef struct
{
    RemotePole_e Left;
    RemotePole_e Right;

} ControlSwitch_t; //ң������s1��s2����

typedef struct
{
    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //����ֵ��
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //����
    } Robot_TargetValue;  //ң�ؼ����������˶��ٶ�
    ControlSwitch_t *ControlSwitch;
    uint16_t infoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־ 
		uint8_t Alldead;
		int16_t ChassisWorkMode;
} DR16_Export_Data_t;         //�������ļ�ʹ�õ�������ݡ�


#define DR16_ExportDataGroundInit \
{                                 \
    {0, 0, 0, 0, 0, 0},           \
    &ControlSwitch,               \
    0,                            \
    0,                            \
}

extern DR16_Export_Data_t DR16_Export_Data;
		
void dr16_getInfo(Can_Export_Data_t RxMessage);

#endif

