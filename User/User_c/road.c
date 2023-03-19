// 该文件用于对道路情况进行判断，并将相应标志位赋值

#include "road.h"
Road road_judge(DG_State *dg_state)
{
	int32 L_Sum;
	L_Sum = dg_state->L_zh_real + dg_state->L_yh_real + dg_state->L_zx_real + dg_state->L_yx_real;
	//tempVar = L_Sum;
	if (1000<L_Sum <= 9000)
	{
		return Stop;
	}
	else if (
		L_Sum >= 11000 &&(
		((dg_state->L_zh_real - 3780) * (dg_state->L_zh_real - 3780) +
		 (dg_state->L_yh_real - 3540) * (dg_state->L_yh_real - 3540) +
		 (dg_state->L_zx_real - 3780) * (dg_state->L_zx_real - 3780) +
		 (dg_state->L_yx_real - 146) * (dg_state->L_yx_real - 146)) <= 2000))
	{
		return Ring_In;
	}
	else if (
		L_Sum >= 11000 &&(
		((dg_state->L_zh_real - 3700) * (dg_state->L_zh_real - 3700) +
		 (dg_state->L_yh_real - 2004) * (dg_state->L_yh_real - 2004) +
		 (dg_state->L_zx_real - 3800) * (dg_state->L_zx_real - 3800) +
		 (dg_state->L_yx_real - 1973) * (dg_state->L_yx_real - 1973)) <= 2000))
	{
		return Ring_Out;
	}
	else if((dg_state->L_zh_real <= 3) &&
		(dg_state->L_yh_real <= 3) &&
		(dg_state->L_zx_real <= 3) &&
		(dg_state->L_yx_real <= 3))
	{
		return Stop;
	}
	else {
		return Stop;
	}
}
