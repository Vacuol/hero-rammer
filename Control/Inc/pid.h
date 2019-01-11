#ifndef _PID_H
#define _PID_H

#include "stm32f4xx_HAL.h"


typedef enum PID_type
{
	positional,
	increment,
}PID_type;


typedef struct
{
	float ref;//���룺ϵͳ���������ĸ���ֵ
	float fdb;//���룺ϵͳ���������ķ���ֵ
	float inte;//����ֵ
	float err[3];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;

	float componentKiMax;

	float output;
	float outputMax;

	PID_type type;

}PID_Regulator_t;


extern PID_Regulator_t cloud_pitch_speed_pid;
extern PID_Regulator_t cloud_pitch_position_pid;
extern PID_Regulator_t cloud_yaw_speed_pid;
extern PID_Regulator_t cloud_yaw_position_pid;
extern PID_Regulator_t rammer_42_ver_pid;
extern PID_Regulator_t rammer_42_pid;
extern PID_Regulator_t rammer_17_pid;
extern PID_Regulator_t underpan_201_pid;
extern PID_Regulator_t underpan_202_pid;
extern PID_Regulator_t underpan_203_pid;
extern PID_Regulator_t underpan_204_pid;

void PID_Calc(PID_Regulator_t *pid);
void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKiMax,float outputMax,PID_type type);
void Cloud_Speed(void);
void Cloud_Position(void);
void Rammer_pid(void);
void Underpan_pid(void);
void ALLPID_Init(void);




#endif
