/**
  *@file timer.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "timer.h"
#include "control.h"
#include "can_my.h"
#include "pid.h"
#include "mpu6050.h"
#include "filter.h"
uint16_t Timetick1ms = 0;
uint16_t TimeElasped=0;
void Timer_interrupt(void)
{
	TIM12->CNT=0;
	
	
	Timetick1ms++;
	MPU6050_GetData();												//get mpu data
	MPUy50Hz.raw_value=mpu6050.Gyro.Radian.y;						//filter get data
	Chebyshev50HzLPF(&MPUy50Hz);									//filter calculate data       	the filtered data is MPUy50Hz.filtered_value
	
	MPUz50Hz.raw_value=mpu6050.Gyro.Radian.z;						//filter get data
	Chebyshev50HzLPF(&MPUz50Hz);									//filter calculate data       	the filtered data is MPUz50Hz.filtered_value

	Cloud_Speed();										//CONTROL cloudmotor 's speed by speed pid
	

	if (Timetick1ms%10==0) {
		Rammer_pid();
		Cloud_Position();
		Underpan_pid();
		PowerControl();
		Rammer_motor_output(rammer_42_pid.output,rammer_17_pid.output,rammer_42_ver_pid.output);
		
	}
	
	cloud_pitch.motor_output=cloud_pitch_speed_pid.output;
	cloud_yaw.motor_output=cloud_yaw_speed_pid.output;
	Cloud_motor_output(cloud_pitch.motor_output,cloud_yaw.motor_output);								//cloudmotor 	
	Underpan_motor_output((int16_t)(underpan_201_pid.output),(int16_t)(underpan_202_pid.output),(int16_t)(underpan_203_pid.output),(int16_t)(underpan_204_pid.output));	
	if (Timetick1ms>999) Timetick1ms=0;
	
	TimeElasped=TIM12->CNT;
}


