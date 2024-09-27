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
  * @brief PID参数初始化
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
  * @brief  PID位置式计算
  */
float pid_calc(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
  
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

/**
  * @brief  PID增量式计算
  */
float pid_calc_shoot(pid_struct_t *pid, float tar, float now)//增量式
{
	pid->tar = tar;
  pid->now = now;
	pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
	
	pid->p_out  = pid->kp * (pid->err[0] - pid->err[1]);//比例部分
  pid->i_out  = pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);//微分部分
	
	LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
	
  pid->delta_u = pid->p_out + pid->i_out + pid->d_out;//控制量
	pid->delta_out = pid->last_delta_out + pid->delta_u;
	
	LIMIT_MIN_MAX(pid->delta_out, -pid->out_max, pid->out_max);
	pid->last_delta_out = pid->delta_out;
  return pid->delta_out;
}

/**
  * @brief  云台6020速度环PID计算
  */
float pid_calc_cloud(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;

	pid->err[0] = ComputeMinOffset(pid->tar,pid->now);//过零处理
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
	pid->err[1] = pid->err[0];
  return pid->output;
}

/**
  * @brief  云台yaw轴6020速度位置环PID计算
  */
float pid_CascadeCalc_yaw(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
	float tar_yaw_outer_angle;
	float_Batwolf(&M6020s_Yaw.target_rotor_angle,&tar_yaw_outer_angle,yaw_target_filter);
	
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//外层角度
 	
	float tar_yaw_inner_speed;
	float_Batwolf(&pid->outer.output,&tar_yaw_inner_speed,yaw_speed_filter);
	
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  云台pitch轴6020速度位置环PID计算
  */
float pid_CascadeCalc_pitch(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
//	float tar_pitch_outer_angle;
//	float_Batwolf(&M6020s_Pitch.target_rotor_angle,&tar_pitch_outer_angle,pitch_target_filter);
	
	pid_calc(&pid->outer,angleTar,angleNow);//外层角度
	
//	float tar_pitch_inner_speed;
//	float_Batwolf(&pid->outer.output,&tar_pitch_inner_speed,pitch_speed_filter);
	
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  电机PID参数初始化
  */
void motor_pid_init()
{	
	pid_init(&motor_pid_Cas_Yaw.inner, 1.7f, 120.0f,220.0f, 1000, 30000);//yaw pid 内环速度, kp=1.7, ki=120,kd=2200, output limit = 30000
	pid_init(&motor_pid_Cas_Yaw.outer, 15.0f , 0.0f,0.0f, 10000, 15000); //yaw pid 外环角度, kp=15 , ki=0, 	kd=0   , output limit = 15000
		
	pid_init(&motor_pid_Cas_Pitch.inner,250.0f , 150.0f, 300.0f, 800, 30000);//pitch pid 内环速度, kp=250, ki=150, kd=300, output limit = 30000
	pid_init(&motor_pid_Cas_Pitch.outer,0.75f, 0   , 0, 30000, 15000);			 //pitch pid 外环角度, kp=0.75,ki=0  , kd=0  , output limit = 15000
	
	pid_init(&motor_pid_shoot[0], 20.0f, 0.6f, 0.0f, 3000, 15000);//shoot pid 速度, kp=20, ki=0.6,kd=0, output limit = 15000
	pid_init(&motor_pid_shoot[1], 20.0f, 0.6f, 0.0f, 3000, 15000);
}

