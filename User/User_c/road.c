//���ļ����ڶԵ�·��������жϣ�������Ӧ��־λ��ֵ


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
