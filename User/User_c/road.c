// ���ļ����ڶԵ�·��������жϣ�������Ӧ��־λ��ֵ

#include "road.h"
Road road_judge(DG_State *dg_state ,Err_Steering *err_steering)
{
	int32 L_Sum;
	L_Sum = dg_state->L_zh_real + dg_state->L_yh_real + dg_state->L_zx_real + dg_state->L_yx_real;
	tempVar = (float)L_Sum;
	if (400 < L_Sum && L_Sum < 5800) // ֱ��
	{
		return Straight;
	}
	else if ( // ��ת
		5800 <= L_Sum &&
		L_Sum < 7800 &&
		err_steering->Err < 0)
	{
		return Curve_Left;
	}
	else if ( // ��ת
		5800 <= L_Sum &&
		L_Sum < 7800 &&
		err_steering->Err >= 0)
	{
		return Curve_Right;
	}
	else if ( // ��Բ��
		7800 <= L_Sum &&
		err_steering->Err < 0)
	{
		return Big_Ring;
	}
	else if ( //������
		L_Sum >= 10000 &&
		((float)((dg_state->L_zh_real - 3780) * (dg_state->L_zh_real - 3780) +
		  (dg_state->L_yh_real - 3540) * (dg_state->L_yh_real - 3540) +
		  (dg_state->L_zx_real - 3780) * (dg_state->L_zx_real - 3780) +
		  (dg_state->L_yx_real - 146) * (dg_state->L_yx_real - 146))/10 <= 8000))
	{
		return Ring_In;
	}
	else if ( //������
		L_Sum >= 10000 &&
		((float)((dg_state->L_zh_real - 3700) * (dg_state->L_zh_real - 3700) +
		  (dg_state->L_yh_real - 2004) * (dg_state->L_yh_real - 2004) +
		  (dg_state->L_zx_real - 3800) * (dg_state->L_zx_real - 3800) +
		  (dg_state->L_yx_real - 1973) * (dg_state->L_yx_real - 1973))/10 <= 8000))
	{
		return Ring_Out;
	}
	
	else if ((dg_state->L_zh_real <= 5) &&
			 (dg_state->L_yh_real <= 5) &&
			 (dg_state->L_zx_real <= 5) &&
			 (dg_state->L_yx_real <= 5))
	{
		return Stop;
	}
	else
	{
		return Straight;
	}
}
