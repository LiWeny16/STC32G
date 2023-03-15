//该文件用于对道路标志位结构体的申明以及同名c文件相应函数的申明

#ifndef __ROAD_H_
#define __ROAD_H_

#include "common.h"
#include "adc.h"

typedef enum
{
  Need_judge,
  No_road,      //严重冲出跑道且无法校正则停车
  Out_of_way,   //已经冲出跑道
  Lose_left,    //丢线
  Lose_right,
  Cross,        //十字交叉
  Straight,//直道
  Curve, //弯道
  LeftRing,//左圆环
  RightRing,//出右圆环
  LeftOutRing,//出左圆环
  RightOutRing,//右圆环
  Three_L,
//  Three_f_cu,
  Three_R,
//  Three_s_cu,
  Finish,//入库标志
  OutGarage,//出库标志
  InGarage,
  Stoped,
}Road;

Road road_judge(DG_State* dg_sta);


#endif