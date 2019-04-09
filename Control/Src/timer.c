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
	Timetick1ms++;


	if (Timetick1ms%5==0) {
		Underpan_pid();	
		
		Underpan_motor_output((int16_t)(underpan_201_pid.output),(int16_t)(underpan_202_pid.output),
		(int16_t)(underpan_203_pid.output),(int16_t)(underpan_204_pid.output));	
	}
	
	if (Timetick1ms%5==1) {
		Cloud_pid();
	}
	
	if (Timetick1ms%5==4){
		LiftMotor_pid();
	
		Lift_motor_output((int16_t)(liftmotor_201_pid.output),(int16_t)(liftmotor_202_pid.output),
		(int16_t)(liftmotor_203_pid.output),(int16_t)(liftmotor_204_pid.output));	
	}

    if (Timetick1ms%5==3){
        CatchMotor_pid();

        CatchYaw_motor_output((int16_t)(catchmoter_right_pid.output),0,(int16_t)(cloud_yaw_pid.output));
    }

    if (Timetick1ms%5==2){
        ProMotor_pid();

        ProPitch_motor_output((int16_t)(promotor_right_pid.output),(int16_t)(promotor_left_pid.output),(int16_t)(cloud_pitch_pid.output));
    }
	
	if (Timetick1ms%10==0){

		JourneyIntagration();
	}
	
	if (Timetick1ms>999) Timetick1ms=0; 
	

}


