#include "pid.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "cloud_control.h"
#include "iir_filter.h"

pid_struct_t motor_pid_shoot[2];
pid_Cascade_t motor_pid_Cas_Yaw;
pid_Cascade_t motor_pid_Cas_Pitch;

float yaw_speed_filter[3] = {0.0f};
float yaw_target_filter[3] = {0.0f};
float pitch_speed_filter[3] = {0.0f};
float pitch_target_filter[3] = {0.0f};

/**
  * @brief PID������ʼ��
  */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

/**
  * @brief  PIDλ��ʽ����
  */
float pid_calc(pid_struct_t *pid, float tar, float now)//λ��ʽ
{
  pid->tar = tar;
  pid->now = now;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
  
  pid->p_out  = pid->kp * pid->err[0];//��������
  pid->i_out += pid->ki * pid->err[0];//���ֲ���
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//΢�ֲ���
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//�޷�
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//������
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

/**
  * @brief  PID����ʽ����
  */
float pid_calc_shoot(pid_struct_t *pid, float tar, float now)//����ʽ
{
	pid->tar = tar;
  pid->now = now;
	pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
	
	pid->p_out  = pid->kp * (pid->err[0] - pid->err[1]);//��������
  pid->i_out  = pid->ki * pid->err[0];//���ֲ���
  pid->d_out  = pid->kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);//΢�ֲ���
	
	LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//�޷�
	
  pid->delta_u = pid->p_out + pid->i_out + pid->d_out;//������
	pid->delta_out = pid->last_delta_out + pid->delta_u;
	
	LIMIT_MIN_MAX(pid->delta_out, -pid->out_max, pid->out_max);
	pid->last_delta_out = pid->delta_out;
  return pid->delta_out;
}

/**
  * @brief  ��̨6020�ٶȻ�PID����
  */
float pid_calc_cloud(pid_struct_t *pid, float tar, float now)//λ��ʽ
{
  pid->tar = tar;
  pid->now = now;

	pid->err[0] = ComputeMinOffset(pid->tar,pid->now);//���㴦��
  pid->p_out  = pid->kp * pid->err[0];//��������
  pid->i_out += pid->ki * pid->err[0];//���ֲ���
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//΢�ֲ���
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//�޷�
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//������
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
	pid->err[1] = pid->err[0];
  return pid->output;
}

/**
  * @brief  ��̨yaw��6020�ٶ�λ�û�PID����
  */
float pid_CascadeCalc_yaw(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//�������������������Ϊ�ڲ�������Ĳο�ֵ���м���
	float tar_yaw_outer_angle;
	float_Batwolf(&M6020s_Yaw.target_rotor_angle,&tar_yaw_outer_angle,yaw_target_filter);
	
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//���Ƕ�
 	
	float tar_yaw_inner_speed;
	float_Batwolf(&pid->outer.output,&tar_yaw_inner_speed,yaw_speed_filter);
	
	pid_calc(&pid->inner,pid->outer.output,speedNow);//�ڲ��ٶ�
	pid->output=pid->inner.output;//������Ϊ�ڲ�����������
	return pid->output;
}

/**
  * @brief  ��̨pitch��6020�ٶ�λ�û�PID����
  */
float pid_CascadeCalc_pitch(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//�������������������Ϊ�ڲ�������Ĳο�ֵ���м���
//	float tar_pitch_outer_angle;
//	float_Batwolf(&M6020s_Pitch.target_rotor_angle,&tar_pitch_outer_angle,pitch_target_filter);
	
	pid_calc(&pid->outer,angleTar,angleNow);//���Ƕ�
	
//	float tar_pitch_inner_speed;
//	float_Batwolf(&pid->outer.output,&tar_pitch_inner_speed,pitch_speed_filter);
	
	pid_calc(&pid->inner,pid->outer.output,speedNow);//�ڲ��ٶ�
	pid->output=pid->inner.output;//������Ϊ�ڲ�����������
	return pid->output;
}

/**
  * @brief  ���PID������ʼ��
  */
void motor_pid_init()
{	
	pid_init(&motor_pid_Cas_Yaw.inner, 1.7f, 120.0f,220.0f, 1000, 30000);//yaw pid �ڻ��ٶ�, kp=1.7, ki=120,kd=2200, output limit = 30000
	pid_init(&motor_pid_Cas_Yaw.outer, 15.0f , 0.0f,0.0f, 10000, 15000); //yaw pid �⻷�Ƕ�, kp=15 , ki=0, 	kd=0   , output limit = 15000
		
	pid_init(&motor_pid_Cas_Pitch.inner,250.0f , 150.0f, 300.0f, 800, 30000);//pitch pid �ڻ��ٶ�, kp=250, ki=150, kd=300, output limit = 30000
	pid_init(&motor_pid_Cas_Pitch.outer,0.75f, 0   , 0, 30000, 15000);			 //pitch pid �⻷�Ƕ�, kp=0.75,ki=0  , kd=0  , output limit = 15000
	
	pid_init(&motor_pid_shoot[0], 20.0f, 0.6f, 0.0f, 3000, 15000);//shoot pid �ٶ�, kp=20, ki=0.6,kd=0, output limit = 15000
	pid_init(&motor_pid_shoot[1], 20.0f, 0.6f, 0.0f, 3000, 15000);
}

