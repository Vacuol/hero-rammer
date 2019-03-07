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

CAN_TxHeaderTypeDef  Tx1Message;		//�������ò���
CAN_RxHeaderTypeDef  Rx1Message;		//�������ò���

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
//    canfilter.FilterNumber = 0;
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
	canfilter.FilterBank=0;
//    canfilter.FilterNumber = 0;
//    hcan1.pTxMsg = &Tx1Message;
//    hcan1.pRxMsg = &Rx1Message;
  

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}

/***************************************
���̵��id��201��204
��̨���id��205��206
�������id��207
***************************************/

void CAN_Getdata(CAN_HandleTypeDef *hcan,CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
	switch(pHeader->StdId)
  {
    case 0x201:
    {
			underpan_para[0].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[0].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[0].motor_current=aData[4]<<8|aData[5];
			underpan_para[0].motor_temperature=aData[6];		
						
			
    }break;
    case 0x202:
    {
			underpan_para[1].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[1].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[1].motor_current=aData[4]<<8|aData[5];
			underpan_para[1].motor_temperature=aData[6];

    }break;
    case 0x203:
    {
			underpan_para[2].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[2].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[2].motor_current=aData[4]<<8|aData[5];
			underpan_para[2].motor_temperature=aData[6];		
			
	
    }break;
    case 0x204:
    {
			underpan_para[3].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[3].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[3].motor_current=aData[4]<<8|aData[5];
			underpan_para[3].motor_temperature=aData[6];	

    }break;
    case 0x205:
	{
 		rammer_42.mechanical_angle=aData[0]<<8|aData[1];
		rammer_42.speed=aData[2]<<8|aData[3];
					
	}break;	
    case 0x206:
	{
 		rammer_17.mechanical_angle=aData[0]<<8|aData[1];
		rammer_17.speed=aData[2]<<8|aData[3];
					
	}break;
    case 0x207:
	{
 		rammer_42_ver.mechanical_angle=aData[0]<<8|aData[1];
		rammer_42_ver.speed=aData[2]<<8|aData[3];
					
	}break;	
    case 0x209:
	{
		  cloud_pitch.mechanical_angle=aData[0]<<8|aData[1];
			cloud_pitch.torque=aData[2]<<8|aData[3];
			cloud_pitch.torque_current=aData[4]<<8|aData[5];
		  if(cloud_pitch.mechanical_angle<4096) cloud_pitch.Bmechanical_angle=cloud_pitch.mechanical_angle;
		  if(cloud_pitch.mechanical_angle>4096) cloud_pitch.Bmechanical_angle=cloud_pitch.mechanical_angle-8192;
						
	}break;
	case 0x20A:
	{
			cloud_yaw.mechanical_angle=aData[0]<<8|aData[1];
			cloud_yaw.torque=aData[2]<<8|aData[3];
			cloud_yaw.torque_current=aData[4]<<8|aData[5];
		  if(cloud_yaw.mechanical_angle<4096) cloud_yaw.Bmechanical_angle=cloud_yaw.mechanical_angle;
		  if(cloud_yaw.mechanical_angle>4096) cloud_yaw.Bmechanical_angle=cloud_yaw.mechanical_angle-8192;
	
	}break;	
				
  }
	
}


/*��������
���̷�������ʱ����ʶ��Ϊ0x200*/
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

/*��������
��̨��������ʱ����ʶ��Ϊ0x2ff*/
void Cloud_motor_output(int16_t iq1,int16_t iq2)
{
	Tx1Message.StdId = 0x2ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;

	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}


/*��������������
��ʶ��Ϊ0x1ff*/
void Rammer_motor_output(int16_t iq1,int16_t iq2,int16_t iq3)
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




