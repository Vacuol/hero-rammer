/**
  *@file can.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "can_my.h"
#include "control.h"

uint8_t TxData[8];
uint32_t pTxMailbox;

CAN_TxHeaderTypeDef  Tx1Message;		
CAN_RxHeaderTypeDef  Rx1Message;		

void CAN1_Init()						
{
	CAN_FilterTypeDef canfilter;
	
	//canfilter.FilterNumber = 0;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//  //filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;
  
	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	  //use different filter for can1&can2
	canfilter.FilterBank=0;
//    hcan1.pTxMsg = &Tx1Message;
//    hcan1.pRxMsg = &Rx1Message;
  

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}

void CAN2_Init()						
{
CAN_FilterTypeDef canfilter;
	
	//canfilter.FilterNumber = 14;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//  //filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;
  
	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	  //use different filter for can1&can2
	canfilter.FilterBank=14;

//    hcan1.pTxMsg = &Tx1Message;
//    hcan1.pRxMsg = &Rx1Message;
  

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan2);
}

/***************************************
can1:
underpan1,2,3,4
catchmotor 5,6
cloudyaw 7

can2:
liftmotor1,2,3,4
prolongmotor 5,6
cloudpitch 7

***************************************/

void CAN1_Getdata(CAN_HandleTypeDef *hcan,CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
	switch(pHeader->StdId)
  {
    case 0x201:
    {
			underpan[0].mechanical_angle=aData[0]<<8|aData[1];
			underpan[0].rotation_rate=aData[2]<<8|aData[3];
			underpan[0].motor_current=aData[4]<<8|aData[5];
			underpan[0].motor_temperature=aData[6];		
						
			
    }break;
    case 0x202:
    {
			underpan[1].mechanical_angle=aData[0]<<8|aData[1];
			underpan[1].rotation_rate=aData[2]<<8|aData[3];
			underpan[1].motor_current=aData[4]<<8|aData[5];
			underpan[1].motor_temperature=aData[6];

    }break;
    case 0x203:
    {
			underpan[2].mechanical_angle=aData[0]<<8|aData[1];
			underpan[2].rotation_rate=aData[2]<<8|aData[3];
			underpan[2].motor_current=aData[4]<<8|aData[5];
			underpan[2].motor_temperature=aData[6];		
			
	
    }break;
    case 0x204:
    {
			underpan[3].mechanical_angle=aData[0]<<8|aData[1];
			underpan[3].rotation_rate=aData[2]<<8|aData[3];
			underpan[3].motor_current=aData[4]<<8|aData[5];
			underpan[3].motor_temperature=aData[6];	

    }break;
	case 0x205:
    {
			catch_right.mechanical_angle=aData[0]<<8|aData[1];
			catch_right.rotation_rate=aData[2]<<8|aData[3];
			catch_right.motor_current=aData[4]<<8|aData[5];
			catch_right.motor_temperature=aData[6];	

    }break;
	case 0x206:
    {
			catch_left.mechanical_angle=aData[0]<<8|aData[1];
			catch_left.rotation_rate=aData[2]<<8|aData[3];
			catch_left.motor_current=aData[4]<<8|aData[5];
			catch_left.motor_temperature=aData[6];	
    }break;
	case 0x207:
    {
			cloud_yaw.mechanical_angle=aData[0]<<8|aData[1];
			cloud_yaw.speed=aData[2]<<8|aData[3];	
    }break;

				
  }
	
}


/***************************************
can1:
underpan1,2,3,4
catchmotor 5,6
cloudyaw 7

can2:
liftmotor1,2,3,4
prolongmotor 5,6
cloudpitch 7

***************************************/
void CAN2_Getdata(CAN_HandleTypeDef *hcan,CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
	switch(pHeader->StdId)
  {
    case 0x201:
    {
			liftmotor[0].mechanical_angle=aData[0]<<8|aData[1];
			liftmotor[0].rotation_rate=aData[2]<<8|aData[3];
			liftmotor[0].motor_current=aData[4]<<8|aData[5];
			liftmotor[0].motor_temperature=aData[6];		
						
			
    }break;
    case 0x202:
    {
			liftmotor[1].mechanical_angle=aData[0]<<8|aData[1];
			liftmotor[1].rotation_rate=aData[2]<<8|aData[3];
			liftmotor[1].motor_current=aData[4]<<8|aData[5];
			liftmotor[1].motor_temperature=aData[6];

    }break;
    case 0x203:
    {
			liftmotor[2].mechanical_angle=aData[0]<<8|aData[1];
			liftmotor[2].rotation_rate=aData[2]<<8|aData[3];
			liftmotor[2].motor_current=aData[4]<<8|aData[5];
			liftmotor[2].motor_temperature=aData[6];		
			
	
    }break;
    case 0x204:
    {
			liftmotor[3].mechanical_angle=aData[0]<<8|aData[1];
			liftmotor[3].rotation_rate=aData[2]<<8|aData[3];
			liftmotor[3].motor_current=aData[4]<<8|aData[5];
			liftmotor[3].motor_temperature=aData[6];	

    }break;
	case 0x205:
    {
			promotor_right.mechanical_angle=aData[0]<<8|aData[1];
			promotor_right.speed=aData[2]<<8|aData[3];	

    }break;
	case 0x206:
    {
			promotor_left.speed=aData[2]<<8|aData[3];
			promotor_left.mechanical_angle=aData[0]<<8|aData[1];
				
    }break;
	case 0x207:
    {
			cloud_pitch.mechanical_angle=aData[0]<<8|aData[1];
			cloud_pitch.speed=aData[2]<<8|aData[3];	
    }break;
				
  }
	
}


void Underpan_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	TxData[6] = iq4 >> 8;
	TxData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}

void Lift_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	TxData[6] = iq4 >> 8;
	TxData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan2, &Tx1Message,  TxData, &pTxMailbox);
}


void CatchYaw_motor_output(int16_t iq1,int16_t iq2,int16_t iq3)
{
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}



void ProPitch_motor_output(int16_t iq1,int16_t iq2,int16_t iq3)
{
	
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;

	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;

	
	HAL_CAN_AddTxMessage(&hcan2, &Tx1Message,  TxData, &pTxMailbox);
}




