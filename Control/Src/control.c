/**
  *@file control.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "control.h"
#include "mpu6050.h"
#include "pid.h"
#include <string.h>

/************TELE***************/
uint8_t teledata_rx[18];
struct telecon_data tele_data;

///************UNDERPAN***************/
//struct underpan_parameter underpan_para[4];
//float underpan_P;//ok
//float underpan_I;//ok
//float IOUT_P;
//float IOUT_I;
struct POWER ph;						//PH means power & heat

/************CLOUD***************/
struct cloud_parameter cloud_pitch,cloud_yaw;
int16_t pitch;
int16_t yaw;


/************DAN***************/
struct rammer_parameter rammer_42;
struct rammer_parameter rammer_17;
struct rammer_parameter rammer_42_ver;



/************CAMERA***************/
struct CAMERA camera;

/************CAPCITY***************/
extern ADC_HandleTypeDef hadc3;
float CurrentCapVoltage = 0;
float LastPower = 0;
float PreviousPower = 0;
float SpeedOfDischarge;
#define PowerMax 50


//裁判系统
struct JUDGE judge;

RxPID rxPID;
uint8_t flag=1;

/****************************************/
/****************function****************/
/****************************************/

/*****接收遥控器数据*****/
void telecontroller_data(void)
{	
//	uint8_t w,a,s,d,press_l,press_r,shift;
//	int16_t speed;

	tele_data.ch0=((teledata_rx[0]| (teledata_rx[1] << 8)) & 0x07ff)-1024 ;						//右摇杆：左右
	tele_data.ch1=(((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff)-1024;				//右摇杆：上下
	tele_data.ch2=(((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff)-1024;			//左：左右
	tele_data.ch3=(((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff)-1024; 				//左：上下
	tele_data.s1=((teledata_rx[5] >> 4)& 0x000C) >> 2;					//左上开关：上中下对应132
	tele_data.s2=((teledata_rx[5] >> 4)& 0x0003);						//右上
	tele_data.x=teledata_rx[6] | (teledata_rx[7] << 8);
	tele_data.y=teledata_rx[8] | (teledata_rx[9] << 8); 
	tele_data.z=teledata_rx[10] | (teledata_rx[11] << 8);
	tele_data.press_l=teledata_rx[12];
	tele_data.press_r=teledata_rx[13];
	tele_data.key=teledata_rx[14] | (teledata_rx[15] << 8);
	tele_data.resv=teledata_rx[16]|(teledata_rx[17]<<8);
	
	
			pitch=pitch_mid+1.0*tele_data.ch1;
			yaw=yaw_mid-1.0*tele_data.ch0;
	
			if(pitch<(pitch_mid-600))pitch=pitch_mid-600;
			if(pitch>(pitch_mid+600))pitch=pitch_mid+600;
			if(yaw<(yaw_mid-800))yaw=yaw_mid-800;
			if(yaw>(yaw_mid+800))yaw=yaw_mid+800;
	
	

		
}
	
void Judge_Getdata()
{	
	if (judge.ID==POWERHEAT)
	{
    		
		ph.data[ph.counter] = judge.recieve[0];
		ph.counter++;
		if (ph.counter == 20)
		{
			ph.counter = 0;
			judge.count=0;
			memcpy(&ph.volt,&ph.data[0],4);
			memcpy(&ph.current,&ph.data[4],4);
			memcpy(&ph.power,&ph.data[8],4);
			memcpy(&ph.power_Buffer,&ph.data[12],4);
			memcpy(&ph.heat_17,&ph.data[16],2);
			memcpy(&ph.heat_42,&ph.data[18],2);
		}
	}
	if (judge.ID==0) judge.count=0;
	
}


void PowerControl()
{
    static uint8_t status = 0;
	static uint8_t cnt = 0;
	float err=0.2;
	power_control_pid.fdb=ph.power;
	power_control_pid.ref=PowerMax;
	HAL_ADC_Start_IT(&hadc3);
#if 0
	if(switche)//手动开关
	{
		charge();
		underpan_201_pid.outputMax=1000;
		underpan_202_pid.outputMax=1000;
		underpan_203_pid.outputMax=1000;
		underpan_204_pid.outputMax=1000;
	}
	else
	{
        if(CurrentCapVoltage>8)
		    discharge();//改速度限幅
		underp an_201_pid.outputMax=3000;
		underpan_202_pid.outputMax=3000;
		underpan_203_pid.outputMax=3000;
		underpan_204_pid.outputMax=3000;
	}
#endif
	
	
	if(ph.power<PowerMax&&flag==1)//正常状态功率大于1W且小于80W
	{
		cnt=0;
		if (status==0) 
        {
			PID_Calc(&power_control_pid);
            TIM5->CCR2 = 0;  //停止放电  
		    if(power_control_pid.output>=0)
				TIM5->CCR1=power_control_pid.output;
			else
				TIM5->CCR1=0;
        }
		else
		{
			TIM5->CCR1 =  0;  //停止充电 
		}
	}
	else
	{
		cnt++;
		if(cnt>=3)
		{
			//cnt=0;
			if(CurrentCapVoltage<0.5)
			{
				flag=1;
				TIM5->CCR2 =  0;  //停止放电
				TIM5->CCR1 =  100;//开始充电
			}
			else
			{
				flag=0;
				TIM5->CCR1 =  0;  //停止充电
				TIM5->CCR2=2000;//开始放电
			}
		}
	}	
    PreviousPower = LastPower;
    LastPower = ph.power;
    if (CurrentCapVoltage>3.2)
    {
		status = 1; //已充满
    }
    else
    {
        status = 0; //未充满
    }
}

/*参数统一初始化*/	
void para_init(void)
{
	ALLPID_Init();

	pitch=pitch_mid;
	yaw=yaw_mid;

}


