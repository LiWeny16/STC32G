
#ifndef __TEMPVAR_H_
#define __TEMPVAR_H_
#include "common.h"
#include "zf_uart.h"

extern volatile float tempVar;
void send_data_sw(int16 a,int16 b,int16 c,int16 d, uint8 target);
///extern volatile uint16 ringInFlag;

#endif