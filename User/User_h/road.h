//该文件用于对道路标志位结构体的申明以及同名c文件相应函数的申明

#ifndef __ROAD_H_
#define __ROAD_H_

#include "common.h"
#include "adc.h"
#include "TempVar.h"

typedef enum
{
  Need_judge,
	Stop,
	Curve_Left,
	Curve_Right,
	Ring_In,
	Ring_Out,
	Big_Ring,
	
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
typedef struct // 增量式PID（电机用）结构体
{

	int16 Ring_In_Flag;
	int16 Ring_Out_Flag;
	int16 Near_Flag;

}FLAG;
Road road_judge(FLAG *road_flag,DG_State *dg_sta ,Err_Steering *err_steering);
int16 isNear(DG_State *dg_state,int16 zh,int16 yh,int16 zx,int16 yx,int16 bear);

#endif