// ���ļ����ڶԵ�·��������жϣ�������Ӧ��־λ��ֵ

#include "road.h"
Road road_judge(TIMER *timer,FLAG *road_flag,DG_State *dg_state ,Err_Steering *err_steering)
{
	int32 L_Sum;
	L_Sum = dg_state->L_zh_real + dg_state->L_yh_real + dg_state->L_zx_real + dg_state->L_yx_real;
	tempVar = (float)L_Sum;
	P41 = 1;
	P52= 1;
	//********************************************************
	if(timer->time1_0>0){
		timer->time1_0--;
		if(timer->time1_0>0 && timer->time1_0<=114){ //��ʱ25
			return  Ring_In;
		}
		if(timer->time1_0<=0){
		  timer->time1_0 =0;
		}
		
	}
	if(timer->time0_0>=20){ //��ʱ���ˣ������� �ж�20msһ�� ����һ��20ms 1s 50��
		if(
				road_flag->Cross_Flag_Last==1 && 
				L_Sum >= 10500 &&
				isNear(&dg_state, 2661, 1965, 3688, 3634,15000) == 0
		){
			road_flag->Cross_Flag++;
			road_flag->Cross_Flag_Last=0;
		}
		//�����ʱ��
		timer->time0_0=0;
		timer->time0_1=0;
	}	
	//*********************************************************
	
	if (400 < L_Sum && L_Sum < 5000) // ֱ��
	{
		return Straight;
	}
	else if ( // ��ת
		5000 <= L_Sum &&
		L_Sum < 9500 &&
		err_steering->Err >=0.5)
	{
		return Curve_Left;
	}
	else if ( // ��ת
		5000 <= L_Sum &&
		L_Sum < 9500 &&
		err_steering->Err <= -0.5)
	{
		return Curve_Right;
	}
	// else if ( // ��Բ����
	// 	7000 <= L_Sum &&
	// 	err_steering->Err < 0)
	// {
	// 	return Big_Ring;
	// }
	else if ( //������
		L_Sum >= 10000 &&
		L_Sum <= 11200 &&
		dg_state->L_yx_real <=500 &&
		isNear(&dg_state,3699,3661,3577,35,12000)==1
		)
	{
			P41=0;
		timer->time1_0=120; //*20ms
		if(road_flag->Ring_Out_Flag ==0){//��ͬʱΪ1������û�߹���Բ��
			road_flag->Ring_In_Flag=1;
	  		return Ring_In;
		}else{
			// �����߹��ˣ��Թ�����
			return Straight;
		}
	}
	else if ( //������
		L_Sum >= 11500 &&
		dg_state->L_yh_real <=1650 &&
		isNear(&dg_state,3681,1512,3701,3564,12000)==1
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
		L_Sum >= 11000 &&
		isNear(&dg_state, 2661, 1965, 3688, 3634,13000) == 1)
	{
	
		road_flag->Ring_In_Flag = 0;
		road_flag->Ring_Out_Flag = 0;
		road_flag->Cross_Flag_Last = 1;
		//��ʱ��*******************
		timer->time0_0++;
		if(timer->time0_0>=500){
			timer->time0_0=0;
			timer->time0_1++;
			timer->time0_1=timer->time0_1>=50?0:timer->time0_1;//��ֹ���
		}
	
		//��ʱ��*******************
		return Force_Straight;
	}
	else if (
			 (dg_state->L_zh_real <= 6) && (dg_state->L_zh_real >=0)  || //��ȫ���� �߼������ȼ���
			 (dg_state->L_yh_real <= 6) && (dg_state->L_yh_real >=0)  ||
			 (dg_state->L_zx_real <= 6) && (dg_state->L_zx_real >=0)  ||
			 (dg_state->L_yx_real <= 6) && (dg_state->L_yx_real >=0) )
	{
	//	tempVar1=2333.0;
		return Stop;
	}

	else
	{
	//	tempVar1=3222;
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