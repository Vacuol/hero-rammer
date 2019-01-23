#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx_HAL.h"
#include "pid.h"

#define pitch_mid  -2000					//云台pitch轴初值   800				-800black
#define yaw_mid -2600						//云台yaw轴初值			300		1000black

/*****ID OF JUDGE****/
#define POWERHEAT 0x04

/*****teler*******/
struct telecon_data
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	uint8_t s1;
	uint8_t s2;
	
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
	
	uint16_t key;   

	uint16_t resv;
};	

///*************underpan**************//
struct underpan_parameter 
{
	uint16_t mechanical_angle;
	int16_t rotation_rate;
	int16_t motor_current;
	uint16_t motor_temperature;
};

typedef struct POWER
{
	uint8_t data[20];
	uint8_t counter;
	float volt;
	float current;
	float power;
	float power_Buffer;
	uint16_t heat_17;
	uint16_t heat_42;
}POWER;

typedef struct cloud_parameter 
{
	uint16_t mechanical_angle;//机械角度
	int16_t Bmechanical_angle;//变换后角度
	int16_t torque;//转矩电流测量
	int16_t torque_current;//转矩电流给定
	int16_t iout;

	int16_t motor_output;
	
} cloud_parameter;

typedef struct rammer_parameter 
{
	uint16_t mechanical_angle;//机械角度
	
	int16_t speed;//转矩电流测量
	int16_t torque_current;//转矩电流给定
	
	int16_t error[3];
	int16_t motor_output;
} rammer_parameter;

typedef struct CAMERA
{
    uint8_t recieve[1];
    uint8_t count;
    uint8_t transmit[1];
    int16_t x;
    int16_t y;
    uint8_t sum;
} CAMERA;

typedef struct JUDGE
{
    uint8_t recieve[1];
    uint8_t count;
    uint8_t transmit[1];
	uint8_t ID;
} JUDGE;

typedef struct
{
    uint8_t Count;
	uint8_t Buf[20];
	uint8_t		Sum;
	uint8_t		pidReadBuf;
	PID_Regulator_t* 	pidAdjust;
} RxPID;



extern RxPID rxPID;
extern uint8_t teledata_rx[18];
extern struct telecon_data tele_data;
extern struct underpan_parameter underpan_para[4];
extern struct POWER ph;
extern struct cloud_parameter cloud_pitch,cloud_yaw;
extern struct rammer_parameter rammer_42;
extern struct rammer_parameter rammer_17;
extern struct rammer_parameter rammer_42_ver;
extern struct CAMERA camera;
extern struct JUDGE judge;
extern int16_t pitch;
extern int16_t yaw;

extern PID_Regulator_t power_control_pid;

//****************function*****************//
void telecontroller_data(void);
void underpan_pid(void);
void cloud_y_v_pid(void);
void cloud_p_v_pid(void);
void PowerControl(void);
void para_init(void);
void Bodan_pid(void);
void Judge_Getdata(void);

#endif
