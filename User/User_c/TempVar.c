#include "TempVar.h"

volatile float tempVar;
volatile float tempVar1;
volatile float tempVar2;
//volatile uint16 ringInFlag;

void send_data_sw(int16 a,int16 b,int16 c,int16 d, uint8 target)
{
    uint8 data_sssa[14];
        //按协议发送指令
    uint8 sc=0,ac=0,i;
    data_sssa[0]=0XAA;
    data_sssa[1]=0XFF;
    data_sssa[2]=target;
    data_sssa[3]=8;

    data_sssa[5]=(*((char *)(&a)));
    data_sssa[4]=(*((char *)(&a)+1));
//    data_sssa[6]=(*((char *)(&a)+2));
//    data_sssa[7]=(*((char *)(&a)+3));

    data_sssa[7]=(*((char*)(&b)));
    data_sssa[6]=(*((char*)(&b)+1));
//    data_sssa[10]=(*((char*)(&b)+2));
//    data_sssa[11]=(*((char*)(&b)+3));

    data_sssa[9]=(*((char*)(&c)));
    data_sssa[8]=(*((char*)(&c)+1));
//    data_sssa[14]=(*((char*)(&c)+2));
//    data_sssa[15]=(*((char*)(&c)+3));

    data_sssa[11]=(*((char*)(&d)));
    data_sssa[10]=(*((char*)(&d)+1));
//    data_sssa[18]=(*((char*)(&d)+2));
//    data_sssa[19]=(*((char*)(&d)+3));

    for(i=0;i<data_sssa[3]+4;i++)
    {
        sc+=data_sssa[i];
        ac+=sc;
    }
    data_sssa[12]=sc;
    data_sssa[13]=ac;
    uart_putbuff(UART_1, data_sssa, 14);

}