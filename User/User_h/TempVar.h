
#ifndef __TEMPVAR_H_
#define __TEMPVAR_H_
#include "common.h"
#include "zf_uart.h"

extern volatile float tempVar;


typedef struct//时间结构体
{
	int time0_0;
	int time0_1;
	int time1_0;
	int time1_1;
}TIMER; 

void send_data_sw(int16 a,int16 b,int16 c,int16 d, uint8 target);
///extern volatile uint16 ringInFlag;

#endif