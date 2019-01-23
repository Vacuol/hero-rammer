#ifndef __MY_USATY_MY_H__
#define __MY_USATY_MY_H__

#include "stm32f4xx_HAL.h"

void sendware(void *wareaddr, uint32_t waresize);

void usart_send_char(uint8_t data);
void UART_SendDataToPC(float *data, uint8_t n);

#endif
