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


/************CLOUD***************/
PID_Regulator_t cloud_pitch_speed_pid;
PID_Regulator_t cloud_pitch_position_pid;
PID_Regulator_t cloud_yaw_speed_pid;
PID_Regulator_t cloud_yaw_position_pid;

PID_Regulator_t cloud_yaw_pid;
PID_Regulator_t cloud_pitch_pid;

PID_Regulator_t underpan_201_pid;
PID_Regulator_t underpan_202_pid;
PID_Regulator_t underpan_203_pid;
PID_Regulator_t underpan_204_pid;

PID_Regulator_t liftmotor_201_pid;
PID_Regulator_t liftmotor_202_pid;
PID_Regulator_t liftmotor_203_pid;
PID_Regulator_t liftmotor_204_pid;

PID_Regulator_t catchmoter_left_pid;
PID_Regulator_t catchmoter_right_pid;

PID_Regulator_t promotor_left_pid;
PID_Regulator_t promotor_right_pid;

PID_Regulator_t power_control_pid;



void PID_Calc(PID_Regulator_t *pid)
{
	if (pid->type == positional)
	{
		pid->err[0] = pid->err[1];
		pid->err[1] = pid->ref - pid->fdb;
		pid->inte += pid->err[1];
		
		
		if(pid->inte > (pid->componentKiMax))
		pid->inte = (pid->componentKiMax);
		else if (pid->inte < -(pid->componentKiMax))
		pid->inte = -(pid->componentKiMax);
		
		
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
	cloud_pitch_speed_pid.fdb = MPUy50Hz.filtered_value;
	cloud_pitch_speed_pid.ref = cloud_pitch_position_pid.output;
	PID_Calc(&cloud_pitch_speed_pid);
	
	//YAW
	cloud_yaw_speed_pid.fdb = MPUx50Hz.filtered_value;
	cloud_yaw_speed_pid.ref = cloud_yaw_position_pid.output;
	PID_Calc(&cloud_yaw_speed_pid);
	
}

void Cloud_Position(void)
{
//	//PITCH
//	cloud_pitch_position_pid.fdb = cloud_pitch.Bmechanical_angle;		
//	cloud_pitch_position_pid.ref = pitch;			
//	PID_Calc(&cloud_pitch_position_pid);
//	
//	//YAW
//	cloud_yaw_position_pid.fdb = cloud_yaw.Bmechanical_angle;	
//	cloud_yaw_position_pid.ref = yaw;
//	PID_Calc(&cloud_yaw_position_pid);
//	
}

void Cloud_pid()
{
	cloud_pitch_pid.fdb = cloud_pitch.speed;
	cloud_pitch_pid.ref = cloud_pitch.set_speed;
	PID_Calc(&cloud_pitch_pid);
	
	cloud_yaw_pid.fdb = cloud_yaw.speed;
	cloud_yaw_pid.ref = cloud_yaw.set_speed;
	PID_Calc(&cloud_yaw_pid);
	
}



void Underpan_pid(void)
{
	underpan_201_pid.fdb=underpan[0].rotation_rate;
	underpan_201_pid.ref=underpan[0].set_speed;
	//underpan_201_pid.ref=0;
	PID_Calc(&underpan_201_pid);

	underpan_202_pid.fdb=underpan[1].rotation_rate;
	underpan_202_pid.ref=underpan[1].set_speed;
	//underpan_202_pid.ref=0;
	PID_Calc(&underpan_202_pid);

	underpan_203_pid.fdb=underpan[2].rotation_rate;
	underpan_203_pid.ref=underpan[2].set_speed;
	//underpan_203_pid.ref=0;
	PID_Calc(&underpan_203_pid);

	underpan_204_pid.fdb=underpan[3].rotation_rate;
	underpan_204_pid.ref=underpan[3].set_speed;
	//underpan_204_pid.ref=0;
	PID_Calc(&underpan_204_pid);
}

void LiftMotor_pid(void)
{
	liftmotor_201_pid.fdb=liftmotor[0].rotation_rate;
	liftmotor_201_pid.ref=liftmotor[0].set_speed;
	//lift_201_pid.ref=0;
	PID_Calc(&liftmotor_201_pid);

	liftmotor_202_pid.fdb=liftmotor[1].rotation_rate;
	liftmotor_202_pid.ref=liftmotor[1].set_speed;
	//liftmotor_202_pid.ref=0;
	PID_Calc(&liftmotor_202_pid);

	liftmotor_203_pid.fdb=liftmotor[2].rotation_rate;
	liftmotor_203_pid.ref=liftmotor[2].set_speed;
	//liftmotor_203_pid.ref=0;
	PID_Calc(&liftmotor_203_pid);

	liftmotor_204_pid.fdb=liftmotor[3].rotation_rate;
	liftmotor_204_pid.ref=liftmotor[3].set_speed;
	//liftmotor_204_pid.ref=0;
	PID_Calc(&liftmotor_204_pid);
}

void CatchMotor_pid(void)
{
    catchmoter_left_pid.fdb = catch_left.rotation_rate;
    catchmoter_left_pid.ref = catch_left.set_speed;
    PID_Calc(&catchmoter_left_pid);

    catchmoter_right_pid.fdb = catch_right.rotation_rate;
    catchmoter_right_pid.ref = catch_right.set_speed;
    PID_Calc(&catchmoter_right_pid);
}

void ProMotor_pid(void)
{
    promotor_left_pid.fdb = promotor_left.speed;
    promotor_left_pid.ref = promotor_left.set_speed;
    PID_Calc(&promotor_left_pid);

    promotor_right_pid.fdb = promotor_right.speed;
    promotor_right_pid.ref = promotor_right.set_speed;
    PID_Calc(&promotor_right_pid);
}


void ALLPID_Init()
{
								    /****kp		ki		kd	    inte	output****/
	PID_Init(&cloud_pitch_position_pid,	2,	0.01,	0,	3000,	800,	positional);

	PID_Init(&cloud_pitch_speed_pid, -2.5,	-0.01,	0.8,	30000,	3000,	positional);

	PID_Init(&cloud_yaw_position_pid,	-2,	-0.05,	0,	3000,	1000,	positional);

	PID_Init(&cloud_yaw_speed_pid,	1.5,	0.02,	1.2,	10000,	3000,	positional);
	
	PID_Init(&underpan_201_pid ,	2,	0.1,	0,	1000,	6000,	increment);

	PID_Init(&underpan_202_pid	,	2,	0.1,	0,	1000,	6000,	increment);

	PID_Init(&underpan_203_pid	,	2,	0.1,	0,	1000,	6000,	increment);

	PID_Init(&underpan_204_pid	, 	2,	0.1,	0,	1000,	6000,	increment);
	
	PID_Init(&liftmotor_201_pid ,	2,	0.1,	0,	1000,	3000,	increment);

	PID_Init(&liftmotor_202_pid	,	2,	0.1,	0,	1000,	3000,	increment);

	PID_Init(&liftmotor_203_pid	,	2,	0.1,	0,	1000,	3000,	increment);

	PID_Init(&liftmotor_204_pid	, 	2,	0.1,	0,	1000,	3000,	increment);

    PID_Init(&catchmoter_left_pid, 	3,	0.4,	0,	1000,	8000,	increment);

    PID_Init(&catchmoter_right_pid, 3,	0.4,	0,	1000,	8000,	increment);

    PID_Init(&promotor_left_pid	, 	2,	1,	0,	1000,	3000,	positional);

    PID_Init(&promotor_right_pid, 	2,	1,	0,	1000,	3000,	positional);
	
	PID_Init(&cloud_yaw_pid	,	2,	0.1,	0,	1000,	2000,	increment);
	
	PID_Init(&cloud_pitch_pid	,	2,	0.1,	0,	1000,	2000,	increment);

	
	PID_Init(&power_control_pid	, 10,	0,	0,	1000,	1800,	increment);
}
