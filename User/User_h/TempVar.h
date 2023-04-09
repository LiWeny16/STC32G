
#ifndef __TEMPVAR_H_
#define __TEMPVAR_H_
#include "common.h"
#include "zf_uart.h"

extern volatile float tempVar;
extern volatile float tempVar1;
extern volatile float tempVar2;

typedef struct//时间结构体
{
	int time0_0;
	int time0_1;
	int time1_0;
	int time1_1;
	int time2_0;
	int time2_1;
}TIMER; 

typedef struct
{
	int speed_counter0_0;
	int speed_counter0_1;
	int speed_counter0_2;
	int speed_counter0_EN;
	int speed_counter1_0;
	int speed_counter1_1;
	int speed_counter1_2;
	int speed_counter1_EN;
}FOOT_COUNTER;

extern volatile int8 EN_Flag;

void wireless_EN(void);

void send_data_sw(int16 a,int16 b,int16 c,int16 d, uint8 target);
///extern volatile uint16 ringInFlag;

#endif