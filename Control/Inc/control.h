#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx_HAL.h"
#include "pid.h"

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

///*************3508 motor**************//
struct underpan_parameter 
{
	uint16_t mechanical_angle;
	int16_t rotation_rate;
	int16_t motor_current;
	uint16_t motor_temperature;
	
	float set_speed;
};

struct Liftmotor_parameter 
{
	uint16_t mechanical_angle;
	int16_t rotation_rate;
	int16_t motor_current;
	uint16_t motor_temperature;
	float set_speed;
	float journey;
	float set_journey;
};

struct Catchmotor_parameter 
{
	uint16_t mechanical_angle;
	int16_t rotation_rate;
	int16_t motor_current;
	uint16_t motor_temperature;
	
	float set_speed;
	float journey;
	float set_journey;
};

///*************2006 motor**************//

typedef struct cloud_parameter 
{
	uint16_t mechanical_angle;//��е�Ƕ�
	int16_t speed;
	int16_t motor_output;
	
	float set_speed;
	float set_journey;
	float journey;
} cloud_parameter;

typedef struct Prolongmotor_parameter 
{
	uint16_t mechanical_angle;//��е�Ƕ�
	int16_t speed;
	int16_t motor_output;
	
	float set_speed;
	float journey;
	float set_journey;
} Prolongmotor_parameter;


typedef struct POWER
{
	uint8_t data[20];
	uint8_t counter;
	uint16_t volt;
	uint16_t current;
	float power;
	uint16_t power_Buffer;
	uint16_t heat_17;
	uint16_t heat_42;
}POWER;

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
	uint8_t	Sum;
	uint8_t	pidReadBuf;
	PID_Regulator_t* 	pidAdjust;
} RxPID;



extern RxPID rxPID;
extern uint8_t teledata_rx[18];
extern struct telecon_data tele_data;
extern struct underpan_parameter underpan[4];
extern struct Liftmotor_parameter liftmotor[4];
extern struct Catchmotor_parameter catch_right;
extern struct Catchmotor_parameter catch_left;
extern struct Prolongmotor_parameter promotor_left,promotor_right;
extern struct cloud_parameter cloud_pitch,cloud_yaw;
extern struct POWER ph;
extern struct JUDGE judge;

extern PID_Regulator_t power_control_pid;


extern uint8_t HoldShift;
extern uint8_t ambulance;
//****************function*****************//
void telecontroller_data(void);
void Server(void);
void PowerControl(void);
void para_init(void);
void Judge_Getdata(void);
void AllPosition(void);
void JourneyIntagration(void);

#endif
