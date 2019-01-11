/**
  *@file timer.h
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */
  
  
#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f4xx_HAL.h"

void Timer_interrupt(void);
void USER_PWM_SetDutyRatio(TIM_HandleTypeDef *htim,uint32_t Channel,uint8_t value);

#endif
