/**
  *@file control.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "control.h"
#include "mpu6050.h"
#include "pid.h"


/************TELE***************/
uint8_t teledata_rx[18];
struct telecon_data tele_data;

///************UNDERPAN***************/
//struct underpan_parameter underpan_para[4];
//float underpan_P;//ok
//float underpan_I;//ok
//float IOUT_P;
//float IOUT_I;

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

//����ϵͳ
struct JUDGE judge;

RxPID rxPID;


/****************************************/
/****************function****************/
/****************************************/

/*****����ң��������*****/
void telecontroller_data(void)
{	
//	uint8_t w,a,s,d,press_l,press_r,shift;
//	int16_t speed;

	tele_data.ch0=((teledata_rx[0]| (teledata_rx[1] << 8)) & 0x07ff)-1024-7;						//��ҡ�ˣ�����
	tele_data.ch1=(((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff)-1024;				//��ҡ�ˣ�����
	tele_data.ch2=(((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff)-1024;			//������
	tele_data.ch3=(((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff)-1024; 				//������
	tele_data.s1=((teledata_rx[5] >> 4)& 0x000C) >> 2;					//���Ͽ��أ������¶�Ӧ132
	tele_data.s2=((teledata_rx[5] >> 4)& 0x0003);						//����
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
	

/*****����pid���Ƴ���*****************
�����ٶ�pid���ڻ��������pid���ƣ��⻷��
***************************************/
//void underpan_pid(void)
//{
//	register uint8_t i;
//	int16_t error;
//	
//	/********��ң�������ݽ���********/
//	underpan_para[0].set_speed=(int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
//	underpan_para[1].set_speed=(int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
//	underpan_para[2].set_speed=(int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
//	underpan_para[3].set_speed=(int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);	
//	
//	
//	/*******��4��pid����********/
//	for(i=0;i<4;i++)
//	{
//		error=underpan_para[i].set_speed-underpan_para[i].rotation_rate;
//		underpan_para[i].i_interg+=underpan_I*error;
//		if(underpan_para[i].i_interg>1000)underpan_para[i].i_interg=1000;
//		if(underpan_para[i].i_interg<-1000)underpan_para[i].i_interg=-1000;
//		underpan_para[i].motor_output=underpan_P*error+underpan_para[i].i_interg;
//		if(underpan_para[i].motor_output>CURRENT_LIM)underpan_para[i].motor_output=CURRENT_LIM;
//		if(underpan_para[i].motor_output<-CURRENT_LIM)underpan_para[i].motor_output=-CURRENT_LIM;
//		
//		error=underpan_para[i].motor_output-underpan_para[i].average_current;
//		underpan_para[i].s_interg+=IOUT_I*error;
//		if(underpan_para[i].s_interg>8000)underpan_para[i].s_interg=8000;
//		if(underpan_para[i].s_interg<-8000)underpan_para[i].s_interg=-8000;
//		underpan_para[i].i_output=IOUT_P*error+underpan_para[i].s_interg;
//		if(underpan_para[i].i_output>CURRENT_MAX)underpan_para[i].i_output=CURRENT_MAX;
//		if(underpan_para[i].i_output<-CURRENT_MAX)underpan_para[i].i_output=-CURRENT_MAX;
//	}	
//}
//	
///**************��̨yaw����pid����***************/
//void cloud_y_v_pid(void)
//{	
//	int16_t err;
//	float output;
//	
//	
//	
//	err=cloud_para[1].Bmechanical_angle-yaw;
//	cloud_para[1].iout+=cloud_y_v_I*err;
//	if(cloud_para[1].iout>1000)cloud_para[1].iout=1000;
//	if(cloud_para[1].iout<-1000)cloud_para[1].iout=-1000;	
//	output=cloud_y_v_P*err+cloud_para[1].iout+cloud_y_v_D*sensor.gyro.radian.x;
//	cloud_para[1].motor_output=(int16_t)output;
//	if(cloud_para[1].motor_output>5000)cloud_para[1].motor_output=5000;
//	if(cloud_para[1].motor_output<-5000)cloud_para[1].motor_output=-5000;		
//}


///**************��̨pitch����pid����***************/
//void cloud_p_v_pid(void)
//{
//	int16_t err;
//	float output;
//	
//	err=cloud_para[0].Bmechanical_angle-pitch;
//	cloud_para[0].iout+=cloud_p_v_I*err;
//	if(cloud_para[0].iout>1000)cloud_para[0].iout=1000;
//	if(cloud_para[0].iout<-1000)cloud_para[0].iout=-1000;	
//	output=cloud_p_v_P*err+cloud_para[0].iout+cloud_p_v_D*sensor.gyro.radian.y;
//	cloud_para[0].motor_output=(int16_t)output;
//	if(cloud_para[0].motor_output>5000)cloud_para[0].motor_output=5000;
//	if(cloud_para[0].motor_output<-5000)cloud_para[0].motor_output=-5000;
//}

///**************�������ٶ�pid����***************/
//void Bodan_pid(void)
//{
//	int16_t err[2];
//	float output;

//	dan_para[0].error[2]=dan_para[0].error[1];			
//	dan_para[0].error[1]=dan_para[0].error[0];		
//	dan_para[0].error[0]=bodan_speed-dan_para[0].speed;
//	err[0]=dan_para[0].error[0]-dan_para[0].error[1];
//	err[1]=dan_para[0].error[0]-2*dan_para[0].error[1]+dan_para[0].error[2];

//	output=dan_P*err[0]+dan_I*dan_para[0].error[0]+dan_D*err[1];
//	dan_para[0].motor_output+=output;
//	if(dan_para[0].motor_output>8000)dan_para[0].motor_output=8000;
//	if(dan_para[0].motor_output<-8000)dan_para[0].motor_output=-8000;	

//}


/*����ͳһ��ʼ��*/	
void para_init(void)
{
	ALLPID_Init();

	pitch=pitch_mid;
	yaw=yaw_mid;

}


