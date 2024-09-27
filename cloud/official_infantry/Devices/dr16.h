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
    RemotePole_UP = 1,  //上
    RemotePole_MID = 3, //中
    RemotePole_DOWM = 2 //下
} RemotePole_e;

typedef struct
{
    RemotePole_e Left;
    RemotePole_e Right;

} ControlSwitch_t; //遥控器的s1、s2拨杆

typedef struct
{
    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    ControlSwitch_t *ControlSwitch;
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志 
		uint8_t Alldead;
		int16_t ChassisWorkMode;
} DR16_Export_Data_t;         //供其他文件使用的输出数据。


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

