//该文件用于对道路情况进行判断，并将相应标志位赋值


#include "road.h"
Road road_judge(DG_State* dg_state)
{
	if(dg_state->L_zh_real>=20 && dg_state->L_yh_real>=20){
	 return Straight;
	}
	else{
	 return Stop;
	}

}
