// ���ļ����ڶԵ�·��������жϣ�������Ӧ��־λ��ֵ

#include "road.h"
Road road_judge(FLAG *road_flag,DG_State *dg_state ,Err_Steering *err_steering)
{
	int32 L_Sum;
	L_Sum = dg_state->L_zh_real + dg_state->L_yh_real + dg_state->L_zx_real + dg_state->L_yx_real;
	tempVar = (float)L_Sum;
	if (400 < L_Sum && L_Sum < 6000) // ֱ��
	{
		return Straight;
	}
	else if ( // ��ת
		6000 <= L_Sum &&
		L_Sum < 7800 &&
		err_steering->Err < 0)
	{
		return Curve_Left;
	}
	else if ( // ��ת
		6000 <= L_Sum &&
		L_Sum < 7800 &&
		err_steering->Err >= 0)
	{
		return Curve_Right;
	}
	else if ( // ��Բ����
		7800 <= L_Sum &&
		err_steering->Err < 0)
	{
		return Big_Ring;
	}
	else if ( //������
		L_Sum >= 11000 &&
		isNear(&dg_state,3779,3775,3740,233,12000)==1
		)
	{
		if(road_flag->Ring_Out_Flag ==0){//��ͬʱΪ1������û�߹���Բ��
			road_flag->Ring_In_Flag=1;
	  		return Ring_In;
		}else{
			// �����߹��ˣ��Թ�����
			return Straight;
		}
	}
	else if ( //������
		L_Sum >= 12000 &&
		isNear(&dg_state,3784,2871,3782,2508,12000)==1
		)
	{
		if(road_flag->Ring_In_Flag ==1){
			road_flag->Ring_Out_Flag =1;//�������Ѿ���Բ�����ʹ���
			return Ring_Out;
		}else{
			return Straight;
		}
	}
	else if ( // ʮ��·��
		L_Sum >= 10500 &&
		isNear(&dg_state, 2660, 2199, 3766, 3158,15000) == 1)
	{
		road_flag->Ring_In_Flag = 0;
		road_flag->Ring_Out_Flag = 0;
		return Straight;
	}
	else if ((dg_state->L_zh_real <= 6) && //��ȫ����
			 (dg_state->L_yh_real <= 6) &&
			 (dg_state->L_zx_real <= 6) &&
			 (dg_state->L_yx_real <= 6))
	{
		return Stop;
	}

	else
	{
		return Straight;
	}
}
int16 isNear(DG_State *dg_state,int16 zh,int16 yh,int16 zx,int16 yx,int16 bear)
{
	if((float)(((int16)dg_state->L_zh_real - zh) * ((int16)dg_state->L_zh_real - zh) +
			 ((int16)dg_state->L_yh_real - yh) * ((int16)dg_state->L_yh_real - yh) +
			 ((int16)dg_state->L_zx_real - zx) * ((int16)dg_state->L_zx_real - zx) +
			 ((int16)dg_state->L_yx_real - yx) * ((int16)dg_state->L_yx_real - yx)) /10 <=bear)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}