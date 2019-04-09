/**
  *@file control.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */
  
#define MIDDLE 3
#define UP 1
#define DOWN 2
#define CATCHSPEEDMAX 400
#define PROLONGSPEEDMAX 6000

#include "control.h"
#include "mpu6050.h"
#include "pid.h"
#include <string.h>


/************TELE***************/
uint8_t teledata_rx[18];
struct telecon_data tele_data;

///************UNDERPAN***************/
struct POWER ph; //PH means power & heat

/************3508***************/
struct underpan_parameter underpan[4];
struct Liftmotor_parameter liftmotor[4];
struct Catchmotor_parameter catch_right,catch_left;

/************2006***************/
struct cloud_parameter cloud_pitch, cloud_yaw;
int16_t pitch;
int16_t yaw;
struct Prolongmotor_parameter promotor_left,promotor_right;

/************CAPCITY***************/
#define PowerMax 80

float CurrentCapVoltage = 0;
uint8_t HoldShift = 0;
uint8_t q,e,w,a,s,d,shift,ctrl;
uint16_t speed;

struct JUDGE judge;

RxPID rxPID;
uint8_t flag = 1;
uint8_t ambulance = 0;
float lift_journey,catch_journey,prolong_journey;
/****************************************/
/****************function****************/
/****************************************/

/*****����ң��������*****/
void telecontroller_data(void)
{
    //	uint8_t w,a,s,d,press_l,press_r,shift;

    tele_data.ch0 = ((teledata_rx[0] | (teledata_rx[1] << 8)) & 0x07ff) - 1024;                                 //
    tele_data.ch1 = (((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff) - 1024;                          //
    tele_data.ch2 = (((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff) - 1024; //
    tele_data.ch3 = (((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff) - 1024;                          //
    tele_data.s1 = ((teledata_rx[5] >> 4) & 0x000C) >> 2;                                                       //
    tele_data.s2 = ((teledata_rx[5] >> 4) & 0x0003);                                                            //
    tele_data.x = teledata_rx[6] | (teledata_rx[7] << 8);
    tele_data.y = teledata_rx[8] | (teledata_rx[9] << 8);
	if (tele_data.x>200) tele_data.x = 0;
	if (tele_data.x<-200) tele_data.x = 0;
	if (tele_data.y>200) tele_data.y = 0;
	if (tele_data.y<-200) tele_data.y = 0;
	
    tele_data.z = teledata_rx[10] | (teledata_rx[11] << 8);
    tele_data.press_l = teledata_rx[12];
    tele_data.press_r = teledata_rx[13];
    tele_data.key = teledata_rx[14] | (teledata_rx[15] << 8);
    tele_data.resv = teledata_rx[16] | (teledata_rx[17] << 8);
	
	w = (tele_data.key & 0x01)>>0;
	s = (tele_data.key & 0x02)>>1;
	a = (tele_data.key & 0x04)>>2;
	d = (tele_data.key & 0x08)>>3;
	q = (tele_data.key & 0x40)>>6;
	e  = (tele_data.key & 0x80)>>7;
	shift = (tele_data.key & 0x10)>>4;
	ctrl = (tele_data.key & 0x20)>>5;

	Server();
}

void Server(void)
{
	
//	if (tele_data.s1==UP&&tele_data.s2==DOWN){
//		liftmotor[0].set_journey = 1400000;
//		liftmotor[1].set_journey = 1400000;
//		liftmotor[2].set_journey = 1400000;
//		liftmotor[3].set_journey = 1400000;
//	}
//	
//	if (tele_data.s1==DOWN&&tele_data.s2==DOWN){
//		liftmotor[0].set_journey = 6000;
//		liftmotor[1].set_journey = 6000;
//		liftmotor[2].set_journey = 6000;
//		liftmotor[3].set_journey = 6000;
//		
//	}

//	
//	if (tele_data.s1==UP&&tele_data.s2==MIDDLE){
//		catch_journey = 60000;
//	}
//	
//	else if (tele_data.s1==DOWN&&tele_data.s2==MIDDLE){
//		catch_journey = 1400;
//	}
//	
//	else {catch_right.set_speed = 0;catch_left.set_speed = 0;}
//	
//	if (tele_data.s1==UP&&tele_data.s2==UP){
//		prolong_journey = 2500000;
//		
//		cloud_yaw.set_journey = 60000;
//		
//	}
//	
//	if (tele_data.s1==DOWN&&tele_data.s2==UP){
//		prolong_journey = 6000;
//		
//		cloud_yaw.set_journey = 1000;
//	}
//	
//	if (tele_data.s1==MIDDLE&&tele_data.s2==UP){
//		promotor_right.set_speed = 0;
//		promotor_left.set_speed = 0;
//	}


//	if (tele_data.s1==DOWN&&tele_data.s2==DOWN){
//	underpan[0].set_speed = (int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*3000);
//	underpan[1].set_speed =	(int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*3000);
//	underpan[2].set_speed =	(int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*3000);
//	underpan[3].set_speed =	(int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*3000);}
//	
//	if (tele_data.s1==MIDDLE&&tele_data.s2==DOWN){
//	liftmotor[0].set_speed = (int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*3000);
//	liftmotor[1].set_speed = (int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*3000);
//	liftmotor[2].set_speed = (int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*3000);
//	liftmotor[3].set_speed = (int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*3000);}
//	
//	if (tele_data.s1==MIDDLE&&tele_data.s2==MIDDLE){
//	liftmotor[0].set_speed = (int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*3000);
//	liftmotor[3].set_speed = (int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*3000);}
//	
//	if (tele_data.s1==MIDDLE&&tele_data.s2==UP){
//	liftmotor[1].set_speed = (int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*3000);
//	liftmotor[2].set_speed = (int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*3000);}
	
	if (shift == 1) speed = 500; 
	else speed = 5000;
	
	//云台控制
	cloud_pitch.set_journey +=   tele_data.y*200;
	cloud_yaw.set_journey -=   tele_data.x*200;
	
	/*正常模式下（左上按钮在下面），按住右键，抬升底盘给英雄补弹，按住左键，打开舱盖
	松开左键就会关闭*/
	if (tele_data.s1==DOWN){
		//底盘运动：wsad控制前后左右，qe控制自转
		underpan[0].set_speed = (int16_t)(3.0*((w-s)+(d-a)-(q-e))*speed);
		underpan[1].set_speed = (int16_t)(3.0*((w-s)-(d-a)-(q-e))*speed);
		underpan[2].set_speed = (int16_t)(3.0*(-(w-s)-(d-a)-(q-e))*speed);
		underpan[3].set_speed = (int16_t)(3.0*(-(w-s)+(d-a)-(q-e))*speed);
		if (tele_data.press_l==1){
			liftmotor[0].set_journey = 900000 ;
			liftmotor[1].set_journey = 900000 ;
			liftmotor[2].set_journey = 900000 ;
			liftmotor[3].set_journey = 900000 ;
		}

		else{
			liftmotor[0].set_journey = 6000 ;
			liftmotor[1].set_journey = 6000 ;
			liftmotor[2].set_journey = 6000 ;
			liftmotor[3].set_journey = 6000 ;
		}
		
		if (tele_data.press_r == 1) 
			TIM4->CCR1=500;					//打开舱盖
		else 
			TIM4->CCR1=1500;					//close cap
	}
	
	//抓弹模式
	if (tele_data.s1==MIDDLE){
		
		catch_journey = 70000;				//把爪子转出去
		
		//底盘运动：wsad控制前后左右(反向)，qe控制自转
		underpan[0].set_speed = (int16_t)(3.0*(-(w-s)-(d-a)-(q-e))*speed);
		underpan[1].set_speed = (int16_t)(3.0*(-(w-s)+(d-a)-(q-e))*speed);
		underpan[2].set_speed = (int16_t)(3.0*((w-s)+(d-a)-(q-e))*speed);
		underpan[3].set_speed = (int16_t)(3.0*((w-s)-(d-a)-(q-e))*speed);
		
		promotor_right.set_speed = -tele_data.ch3*40;
		promotor_left.set_speed = -tele_data.ch3*40;
		
		if (tele_data.press_l == 1) {
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4, GPIO_PIN_RESET); 			//把爪子抓紧
			catch_journey =	4000;				//把爪子转回来
		}
		
		if (tele_data.press_l == 0){
			catch_journey = 60000;				//把爪子转出来	
		}
		
		if (tele_data.press_r == 1){
			HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4, GPIO_PIN_SET);		//把爪子松掉
		}
		
	}
	
	//上岛模式
	if (tele_data.s1==UP){
		underpan[0].set_speed = (int16_t)(3.0*(-(w-s)-(d-a)-(q-e))*speed);
		underpan[1].set_speed = (int16_t)(3.0*(-(w-s)+(d-a)-(q-e))*speed);
		underpan[2].set_speed = (int16_t)(3.0*((w-s)+(d-a)-(q-e))*speed);
		underpan[3].set_speed = (int16_t)(3.0*((w-s)-(d-a)-(q-e))*speed);
		
		if (tele_data.press_l==1 && shift ==1)
		{
			liftmotor[1].set_journey = 900000 ;
			liftmotor[2].set_journey = 900000 ;
		}
		
		if (tele_data.press_l==1 && ctrl ==1)
		{
			liftmotor[1].set_journey = 6000 ;
			liftmotor[2].set_journey = 6000 ;
		}
		
		if (tele_data.press_r==1 && shift ==1)
		{
			liftmotor[0].set_journey = 900000 ;
			liftmotor[3].set_journey = 900000 ;
		}
		
		if (tele_data.press_r==1 && ctrl ==1)
		{
			liftmotor[0].set_journey = 6000 ;
			liftmotor[3].set_journey = 6000 ;
		}
	



		
	}
	
	
	
		
	AllPosition();

}

//改变各电机位置
void AllPosition()
{
	int8_t i;
	
	for (i=0;i<4;i++){
		liftmotor[i].set_journey = lift_journey ; 
		liftmotor[i].set_speed = (liftmotor[i].set_journey - liftmotor[i].journey) / 50 ;
		if (liftmotor[i].set_speed>2000)  liftmotor[i].set_speed=2000;
		if (liftmotor[i].set_speed<-2000)  liftmotor[i].set_speed=-2000;
	}
	
	//catch journey set
	catch_right.set_journey = catch_journey;
	catch_left.set_journey = -catch_journey;
	catch_right.set_speed = (catch_right.set_journey - catch_right.journey) / 30;
	if (catch_right.set_speed > CATCHSPEEDMAX) catch_right.set_speed = CATCHSPEEDMAX;
	if (catch_right.set_speed < -CATCHSPEEDMAX) catch_right.set_speed = -CATCHSPEEDMAX;
	
	catch_left.set_speed = (catch_left.set_journey - catch_left.journey) / 30;
	if (catch_left.set_speed > CATCHSPEEDMAX) catch_left.set_speed = CATCHSPEEDMAX;
	if (catch_left.set_speed < -CATCHSPEEDMAX) catch_left.set_speed = -CATCHSPEEDMAX;
	
//	//prolong journey set
//	promotor_right.set_journey = -prolong_journey;
//	promotor_left.set_journey = -prolong_journey;
//	promotor_right.set_speed = (promotor_right.set_journey - promotor_right.journey) / 30;
//	if (promotor_right.set_speed > PROLONGSPEEDMAX) promotor_right.set_speed = PROLONGSPEEDMAX;
//	if (promotor_right.set_speed < -PROLONGSPEEDMAX) promotor_right.set_speed = -PROLONGSPEEDMAX;
//	
//	promotor_left.set_speed = (promotor_left.set_journey - promotor_left.journey) / 30;
//	if (promotor_left.set_speed > PROLONGSPEEDMAX) promotor_left.set_speed = PROLONGSPEEDMAX;
//	if (promotor_left.set_speed < -PROLONGSPEEDMAX) promotor_left.set_speed = -PROLONGSPEEDMAX;
	
	//pitch
	cloud_pitch.set_speed = (cloud_pitch.set_journey - cloud_pitch.journey) / 20;
	if (cloud_pitch.set_speed > 600) cloud_pitch.set_speed = 600;
	if (cloud_pitch.set_speed < -600) cloud_pitch.set_speed = -600;
	
	//yaw
	cloud_yaw.set_speed = (cloud_yaw.set_journey - cloud_yaw.journey) / 20;
	if (cloud_yaw.set_speed > 600) cloud_yaw.set_speed = 600;
	if (cloud_yaw.set_speed < -600) cloud_yaw.set_speed = -600;
	
}


void JourneyIntagration()
{
	liftmotor[0].journey += liftmotor[0].rotation_rate;
	liftmotor[1].journey += liftmotor[1].rotation_rate;
	liftmotor[2].journey += liftmotor[2].rotation_rate;
	liftmotor[3].journey += liftmotor[3].rotation_rate;
	
	catch_right.journey += catch_right.rotation_rate;
	catch_left.journey = -catch_right.journey;
	
	promotor_right.journey += promotor_right.speed;
	promotor_left.journey = promotor_right.journey;
	
	cloud_pitch.journey += cloud_pitch.speed;
	cloud_yaw.journey += cloud_yaw.speed;
}

/*����ͳһ��ʼ��*/
void para_init(void)
{
    ALLPID_Init();
}

void Judge_Getdata()
{
//    if (judge.ID == POWERHEAT)
//    {

//        ph.data[ph.counter] = judge.recieve[0];
//        ph.counter++;
//        if (ph.counter == 14)
//        {
//            ph.counter = 0;
//            judge.count = 0;
//			ph.volt=(ph.data[0]<<8)+ph.data[1];
//			ph.current=(ph.data[2]<<8)+ph.data[3];
////            memcpy(&ph.volt, &ph.data[0], 4);
////            memcpy(&ph.current, &ph.data[4], 4);
//            memcpy(&ph.power, &ph.data[4], 4);
//			ph.power_Buffer=(ph.data[8]<<8)+ph.data[9];
//			ph.heat_17=(ph.data[10]<<8)+ph.data[11];
//			ph.heat_42=(ph.data[12]<<8)+ph.data[13];
////            memcpy(&ph.power_Buffer, &ph.data[12], 4);
////            memcpy(&ph.heat_17, &ph.data[16], 2);
////            memcpy(&ph.heat_42, &ph.data[18], 2);
//        }
//    }
//    if (judge.ID == 0)
//        judge.count = 0;
}



void PowerControl()
{
//    static uint8_t cnt = 0;
//    power_control_pid.fdb = ph.power;
//    power_control_pid.ref = PowerMax;
//    HAL_ADC_Start_IT(&hadc3);
//	if(tele_data.s1==2) HoldShift = 1;
//	else HoldShift = 0;
//    if (ambulance)
//    {
//        underpan_201_pid.outputMax = 2000;
//        underpan_202_pid.outputMax = 2000;
//        underpan_203_pid.outputMax = 2000;
//        underpan_204_pid.outputMax = 2000;
//    }
//    else
//    {
//		underpan_201_pid.outputMax = 4000;
//        underpan_202_pid.outputMax = 4000;
//        underpan_203_pid.outputMax = 4000;
//        underpan_204_pid.outputMax = 4000;
//        if (HoldShift == 1) //���ٻ�����
//        {
//            if (CurrentCapVoltage > 1.5f) //�����е�ŵ�
//            {
//                //����
//                underpan_201_pid.ref = 8000;
//                underpan_202_pid.ref = 8000;
//                underpan_203_pid.ref = 8000;
//                underpan_204_pid.ref = 8000;

//                underpan_201_pid.outputMax = 8000;
//                underpan_202_pid.outputMax = 8000;
//                underpan_203_pid.outputMax = 8000;
//                underpan_204_pid.outputMax = 8000;

//                HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(Discharge_GPIO_Port, Discharge_Pin, GPIO_PIN_SET);
//            }
//			else
//				HAL_GPIO_WritePin(Discharge_GPIO_Port, Discharge_Pin, GPIO_PIN_RESET);
//        }
//		

//        //û�����ʣ����
//        if (ph.power < PowerMax)
//        {
//            if (HoldShift == 0)
//            {
//                cnt = 0;                      //�����ʴ�������
//                if (CurrentCapVoltage < 3.2f) //����û���� ���ų�
//                {
//                    HAL_GPIO_WritePin(Discharge_GPIO_Port, Discharge_Pin, GPIO_PIN_RESET);
//                    HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);
//                }
//                else //���� ֹͣ��
//                {
//                    HAL_GPIO_WritePin(Discharge_GPIO_Port, Discharge_Pin, GPIO_PIN_RESET);
//                    HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);
//                }
//            }
//        }
//        else //δ֪�������¼�����
//        {
//            cnt++;	//�˳�PID��������ĳ�����
//            if (cnt >= 2) //ȷʵ������
//            {
//                //ֹͣ���
//                HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);
//                //ֹͣ�����Գ�����
//                if (cnt >= 3)
//                {
//                    if (CurrentCapVoltage > 1.5f)
//					{
//                        HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(Discharge_GPIO_Port, Discharge_Pin, GPIO_PIN_SET);
//					}
//                    else
//                    {
//                        underpan_201_pid.outputMax = 2000;
//                        underpan_202_pid.outputMax = 2000;
//                        underpan_203_pid.outputMax = 2000;
//                        underpan_204_pid.outputMax = 2000;
//						
//                        HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(Discharge_GPIO_Port, Discharge_Pin, GPIO_PIN_RESET);
//                        //���԰�ȫ����1s
//                        ambulance = 1;
//                    }
//                }
//            }
//        }
//    }
}





