/**
  *@file usart.c
  *@date 2018-10-11
  *@author Vacuo.W
  *@brief 
  */

#include "usart_my.h"
#include "control.h"
#include <math.h>
#include "pid.h"
#include <string.h>

//struct CAMERA camera;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

uint8_t cheat_ready=1;
uint8_t t;
double angle;

uint8_t RX_PID_Count = 0;
uint8_t RX_PID_Buf[20];
uint8_t		RX_PID_Sum = 0;
uint8_t		pidReadBuf;
uint8_t 	charBuf[4];
PID_Regulator_t 	*pidAdjust;

//串口发送1个字芿
//c:要发送的字符
void usart_send_char(uint8_t c)
{
    while(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TC)==RESET){}; 
    UART4->DR=c;  
} 
// *data:	所发送数据数组，最大长度为6
// n:		发送数据所占字节数
void UART_SendDataToPC(float *data, uint8_t n)
{
	int8_t i = 0;
	unsigned char *point;
	point = (unsigned char*)data;	  //得到float的地址
	if(n > 24)
		n = 24;
	usart_send_char('$');
	usart_send_char('%');
	usart_send_char(n * 0.25);
//	
	for(i = 0; i < n;i ++)
		usart_send_char(point[i]);
}


void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdr, sizeof(cmdr), 5000);
}
uint8_t data[10];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
	/******************裁判系统串口数据处理********************/
	if (huart->Instance==USART6)
	{
		switch (judge.count)
        {
			case 0:
                if (judge.recieve[0]==0xA5) judge.count=1;
                else judge.count=0;
                break;
			case 1:
				judge.count=2;
				break;
			case 2:
				judge.count=3;
				break;
			case 3:
				judge.count=4;
				break;
			case 4:
				judge.count=5;
				break;
			case 5:
				if (judge.recieve[0]==0x02) judge.ID=POWERHEAT;
				else judge.ID = 0;
				judge.count=6;
				break;
			case 6:
				if (judge.ID==POWERHEAT) {
					if (judge.recieve[0]==0x02) judge.ID=POWERHEAT;
					else judge.count=0;
				}
				else judge.ID = 0;
				judge.count=7;
				break;
			case 7:
				Judge_Getdata();
				break;
		}			
					
				
		
	}
	
	/*************PID参数串口数据处理***********/
	else if (huart->Instance == UART4)
	{
		rxPID.Buf[rxPID.Count & 0x7f] = rxPID.pidReadBuf;
		//是否开始接收
		if ((rxPID.Count & 0x7f) == 0 && rxPID.Buf[0] != '$')
			return;

		rxPID.Count++;

		if ((rxPID.Count & 0x7f) == 8)
		{
			//接收正确
			if (rxPID.Sum == rxPID.pidReadBuf)
			{
				for (int i = 0; i < 4; i++)
					charBuf[i] = rxPID.Buf[i + 3];

				switch (rxPID.Buf[1])
				{
				case 'p':
					memcpy(&rxPID.pidAdjust->kp, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->kp = -rxPID.pidAdjust->kp;
					break;
				case 'i':
					memcpy(&rxPID.pidAdjust->ki, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->ki = -rxPID.pidAdjust->ki;
					break;
				case 'd':
					memcpy(&rxPID.pidAdjust->kd, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->kd = -rxPID.pidAdjust->kd;
					break;
				}
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
			else
			{
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
		}
		else
			rxPID.Sum += rxPID.pidReadBuf;
    }
 
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET) {
	
 __HAL_UART_CLEAR_OREFLAG(huart);
			t=huart->Instance->DR;
	}
		
	
}
