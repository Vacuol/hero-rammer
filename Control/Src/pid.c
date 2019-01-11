/**
  *@file timer.c
  *@date 2018-12-17
  *@author Vacuo.W
  *@brief 
  */
  
#include "pid.h"
#include "control.h"
#include "can_my.h"
#include "mpu6050.h"
#include "filter.h"

#define SPEED_MAX 5000

/************CLOUD***************/
PID_Regulator_t cloud_pitch_speed_pid;
PID_Regulator_t cloud_pitch_position_pid;
PID_Regulator_t cloud_yaw_speed_pid;
PID_Regulator_t cloud_yaw_position_pid;
PID_Regulator_t rammer_42_ver_pid;
PID_Regulator_t rammer_42_pid;
PID_Regulator_t rammer_17_pid;
PID_Regulator_t underpan_201_pid;
PID_Regulator_t underpan_202_pid;
PID_Regulator_t underpan_203_pid;
PID_Regulator_t underpan_204_pid;


struct underpan_parameter underpan_para[4];

void PID_Calc(PID_Regulator_t *pid)
{
	if (pid->type == positional)
	{
		pid->err[0] = pid->err[1];
		pid->err[1] = pid->ref - pid->fdb;
		pid->inte += pid->err[1];
		
		
		if(pid->inte > (pid->componentKiMax)/(pid->ki+0.0001))//�Ի������������
		pid->inte = (pid->componentKiMax)/(pid->ki+0.0001);
		else if (pid->inte < -(pid->componentKiMax)/(pid->ki+0.0001))
		pid->inte = -(pid->componentKiMax)/(pid->ki+0.0001);
		
		
		pid->componentKp  = pid->kp * pid->err[1];
		pid->componentKi  = pid->ki * pid->inte;
		pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);
		

		
		pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	}
	else
	{
		pid->err[2] = pid->err[1];
		pid->err[1] = pid->err[0];
		pid->err[0] = pid->ref - pid->fdb;
		pid->output += ((pid->err[0] - pid->err[1]) * pid->kp + (pid->err[0]) * pid->ki + (pid->err[0] - 2 * pid->err[1] + pid->err[2]) * pid->kd);
	}
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;	
}

void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKiMax,float outputMax,PID_type type)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->inte = 0;

	pid->componentKiMax = componentKiMax;

	pid->outputMax = outputMax;

	pid->type = type;
}

void Cloud_Speed(void)
{
	//PITCH
	cloud_pitch_speed_pid.fdb = MPUy50Hz.filtered_value;					//���������������ǽ��ٶ�
	cloud_pitch_speed_pid.ref = cloud_pitch_position_pid.output;			//�趨������λ�û����
	PID_Calc(&cloud_pitch_speed_pid);
	
	//YAW
	cloud_yaw_speed_pid.fdb = MPUz50Hz.filtered_value;					//���������������ǽ��ٶ�
	cloud_yaw_speed_pid.ref = cloud_yaw_position_pid.output;			//�趨������λ�û����
	PID_Calc(&cloud_yaw_speed_pid);
	
}

void Cloud_Position(void)
{
	//PITCH
	cloud_pitch_position_pid.fdb = cloud_pitch.Bmechanical_angle;				//��������:��̨����Ƕ�
	cloud_pitch_position_pid.ref = pitch;										//�趨����
	PID_Calc(&cloud_pitch_position_pid);
	
	//YAW
	cloud_yaw_position_pid.fdb = cloud_yaw.Bmechanical_angle;					//������������̨����Ƕ�
	cloud_yaw_position_pid.ref = yaw;										//�趨������λ�û����
	PID_Calc(&cloud_yaw_position_pid);
	
}

void Rammer_pid(void)
{
	rammer_42_ver_pid.fdb=rammer_42_ver.speed;
	rammer_42_ver_pid.ref=0;
	PID_Calc(&rammer_42_ver_pid);
	
	rammer_42_pid.fdb=rammer_42.speed;
	rammer_42_pid.ref=0;
	PID_Calc(&rammer_42_pid);
	
	rammer_17_pid.fdb=rammer_17.speed;
	rammer_17_pid.ref=1000;
	PID_Calc(&rammer_17_pid);
}



void Underpan_pid(void)
{

	
	underpan_201_pid.fdb=underpan_para[0].rotation_rate;
	underpan_201_pid.ref=(int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	//underpan_201_pid.ref=0;
	PID_Calc(&underpan_201_pid);

	underpan_202_pid.fdb=underpan_para[1].rotation_rate;
	underpan_202_pid.ref=(int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	//underpan_202_pid.ref=0;
	PID_Calc(&underpan_202_pid);

	underpan_203_pid.fdb=underpan_para[2].rotation_rate;
	underpan_203_pid.ref=(int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	//underpan_203_pid.ref=0;
	PID_Calc(&underpan_203_pid);

	underpan_204_pid.fdb=underpan_para[3].rotation_rate;
	underpan_204_pid.ref=(int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	//underpan_204_pid.ref=0;
	PID_Calc(&underpan_204_pid);

}



void ALLPID_Init()
{
								    /****kp		ki		kd	�����޷�������޷�****/
	PID_Init(&cloud_pitch_position_pid,	-1.5,	0,	0,	300,	800,	positional);

	PID_Init(&cloud_pitch_speed_pid,	3,	0.004,	0,	100,	2000,	positional);
	
	PID_Init(&cloud_yaw_position_pid,	2,	0,	0,	300,	1000,	positional);

	PID_Init(&cloud_yaw_speed_pid	,	5,	0.006,	0,	400,	2000,	positional);
	
	PID_Init(&rammer_42_pid	,	2,	1,	0,	1000, 	2000,	positional);
	
	PID_Init(&rammer_17_pid	,	2,	1,	0,	1000,	2000,	positional);
	
	PID_Init(&rammer_42_ver_pid	,	2,	1,	0,	1000,	2000,	positional);
	
	PID_Init(&underpan_201_pid ,	2,	0.1,	0,	1000,	2000,	positional);

	PID_Init(&underpan_202_pid	,	2,	0.1,	0,	1000,	2000,	positional);

	PID_Init(&underpan_203_pid	,	2,	0.1,	0,	1000,	2000,	positional);

	PID_Init(&underpan_204_pid	, 2,	0.1,	0,	1000,	2000,	positional);
	
	
}
