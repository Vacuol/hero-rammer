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

void Timer_interrupt(void)
{
	Timetick1ms++;
	MPU6050_GetData();												//get mpu data
	MPUy50Hz.raw_value=mpu6050.Gyro.Radian.y;						//filter get data
	Chebyshev50HzLPF(&MPUy50Hz);									//filter calculate data       	the filtered data is MPUy50Hz.filtered_value
	
	MPUz50Hz.raw_value=mpu6050.Gyro.Radian.z;						//filter get data
	Chebyshev50HzLPF(&MPUz50Hz);									//filter calculate data       	the filtered data is MPUy50Hz.filtered_value

	Cloud_Speed();										//CONTROL cloudmotor 's speed by speed pid
	
	/*����*/
	if (Timetick1ms%10==0) {
		Rammer_pid();
		Cloud_Position();
		Underpan_pid();
		//Rammer_motor_output(rammer_42_pid.output,rammer_17_pid.output,rammer_42_ver_pid.output);
		
	}
	
	cloud_pitch.motor_output=cloud_pitch_speed_pid.output;
	cloud_yaw.motor_output=cloud_yaw_speed_pid.output;
	Cloud_motor_output(cloud_pitch.motor_output,(-cloud_yaw.motor_output));								//cloudmotor 	
	//Underpan_motor_output((int16_t)(underpan_201_pid.output),(int16_t)(underpan_202_pid.output),(int16_t)(underpan_203_pid.output),(int16_t)(underpan_204_pid.output));	
	if (Timetick1ms>999) Timetick1ms=0;
	
}


/**
  * @brief  调整PWM占空比
  * @param  value为占空比 value=50 即占空比为50%
  * @retval None
  */
void USER_PWM_SetDutyRatio(TIM_HandleTypeDef *htim,uint32_t Channel,uint8_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	
	uint32_t period=htim->Init.Period+1;
	uint32_t pluse=(value * period)/100;
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pluse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);
	HAL_TIM_PWM_Start(htim, Channel);   
}

