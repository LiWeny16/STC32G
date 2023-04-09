// ���ļ����ڶԵ�·��������жϣ�������Ӧ��־λ��ֵ

#include "road.h"
Road road_judge(FOOT_COUNTER *foot_counter,TIMER *timer,FLAG *road_flag,DG_State *dg_state ,Err_Steering *err_steering)
{
	int32 L_Sum;
	int32 L_Sum_Plus;
	int32 L_Sum_Left;
	int32 L_Sum_Right;
	int32 Ring_L_Min;
	int32 Ring_R_Min;
	Ring_L_Min = 400; //б��н���
	Ring_R_Min = 900;
	L_Sum_Left =  dg_state->L_zh_real +dg_state->L_zs_real+dg_state->L_zx_real;
	L_Sum_Right =  dg_state->L_yh_real +dg_state->L_ys_real+dg_state->L_yx_real;

	
	L_Sum = dg_state->L_zh_real + dg_state->L_yh_real + dg_state->L_zx_real + dg_state->L_yx_real;
	L_Sum_Plus = dg_state->L_zh_real + dg_state->L_yh_real + dg_state->L_zx_real + dg_state->L_yx_real +dg_state->L_zs_real+dg_state->L_ys_real;
	tempVar = (float)L_Sum;
	P41 = 1;
	P52= 1;
	//
	if(!EN_Flag){return Stop;}
	//*******************��************��*********************
	
	
	if(foot_counter->speed_counter0_EN == 1 && foot_counter->speed_counter0_0<=6290){
	  return OutGarage;
	}else{
	  foot_counter->speed_counter0_EN = 0;
	}
	if(road_flag->InGarage_Flag==0 && 
		foot_counter->speed_counter1_1 >42 && 
	  Light_Right==1 && 
		Light_Left==0){
		road_flag->InGarage_Flag=1;
		tempVar2=114.0;
		timer->time2_0 =80;
	  return InGarage;
	}
	//*******************************************************
	
	
	
	//*****************ʱ**************��*************************
	if(timer->time2_0>0){ //���
		timer->time2_0--;
		if(timer->time2_0>0 && timer->time2_0<=79){ 
			return  InGarage;
		}
		if(timer->time2_0<=0){
		  timer->time2_0 =0;
			timer->time2_1 =0;
		}
	}
	if(timer->time1_0>0){//��ʱ���ˣ������� �ж�20msһ�� ����һ��20ms 1s 50��
		timer->time1_0--;
		if(timer->time1_0>0 && timer->time1_0<=119){ //��ʱ25
			return  Ring_In;
		}
		if(timer->time1_0<=0){
		  timer->time1_0 =0;
		}
		
	}
	if(timer->time0_0>=20){ 
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
	
	if (
		(500 < L_Sum && L_Sum < 5000) &&	
		 err_steering->Err <=0.9 &&
	   err_steering->Err >=-0.9 &&
	   dg_state->L_ys_real >Ring_L_Min &&
		 dg_state->L_zs_real >Ring_R_Min 
	) // ֱ��
	{
		return Straight;
	}
	else if ( // ��ת
		500 <= L_Sum &&
		L_Sum < 5800 &&
	  err_steering->Err >0.9 &&
	//	err_steering->Err <=1.2 &&
	  dg_state->L_ys_real >Ring_L_Min 
	 // dg_state->L_zs_real >Ring_R_Min 
		)
	{
		return Curve_Left;
	}
	else if ( // ��ת
		1000 <= L_Sum &&
		L_Sum < 5800 &&
		err_steering->Err <-0.9 &&
	//   err_steering->Err >=-1.2  &&
	//  dg_state->L_ys_real >Ring_L_Min &&
	  dg_state->L_zs_real >Ring_R_Min
		)
	{
		return Curve_Right;
	}
//	Ring_L_Min = 400; //б��н���
//	Ring_R_Min = 900;
	else if(  //����
		(5800 <= L_Sum &&
		L_Sum < 10800 &&
	  dg_state->L_ys_real <=Ring_L_Min)|| //�����180
     (dg_state->L_zs_real>3000 && dg_state->L_ys_real <180)	
	//	err_steering->Err > 0.2
	){
		return Ring_Left;
	}
	else if( //����
		(5800 <= L_Sum &&
		L_Sum < 10800 &&
		dg_state->L_zs_real <=Ring_R_Min)||
		(dg_state->L_zs_real<200 && dg_state->L_ys_real >3000)
	//	err_steering->Err < -0.2
	){
		return Ring_Right;
	}
	// else if ( // ��Բ����
 	//6500 <= L_Sum &&
	// L_Sum<=7980 &&
	// 	err_steering->Err < 0)
	// {
	// 	return Big_Ring;
	// }
	else if ( //������ 
		L_Sum >= 9000 &&
		L_Sum <= 12000 &&
		dg_state->L_yx_real <=900 &&
		dg_state->L_ys_real <=2000 &&
		isNear(&dg_state,3693,3325,3683,718,12000)==1
		)
	{
			P41=0;
		
		if(road_flag->Ring_Out_Flag ==0){//��ͬʱΪ1������û�߹���Բ��
			timer->time1_0=120; //*20ms
			road_flag->Ring_Out_Flag=1; //�´β���
			road_flag->Ring_In_Flag=1;
	  		return Ring_In;
		}else{
			// �����߹��ˣ��Թ�����
			return Force_Straight;
		}
	}
	else if 
	( //������ ��ʱ����
		(
		L_Sum >= 22200 &&
		dg_state->L_zh_real >=3200 &&
		dg_state->L_yh_real <=2500 &&
		isNear(&dg_state,3667,2127,3668,3573,12000)==1 
		) ||
	  (
			L_Sum >= 12000 &&
			isNear(&dg_state,3466,1913,3197,3274,8000)==1
		) ||
		(
			L_Sum >= 13400 &&
			isNear(&dg_state,3667,2788,3668,3566,12000)==1
		)
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
		L_Sum >= 18000 &&
		dg_state->L_zx_real <=1500 &&
		isNear(&dg_state, 1834, 2333, 919, 3446,11000) == 1)
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
			 ((dg_state->L_zh_real <= 10) && (dg_state->L_zh_real >=0))  && //��ȫ���� �߼������ȼ���
			 ((dg_state->L_yh_real <= 10) && (dg_state->L_yh_real >=0))  &&
			 ((dg_state->L_zx_real <= 10) && (dg_state->L_zx_real >=0))  &&
			 ((dg_state->L_yx_real <= 10) && (dg_state->L_yx_real >=0)) )
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

int16 isNearPlus(DG_State *dg_state,int16 zh,int16 yh,int16 zx,int16 yx ,int16 zs,int16 ys,int16 bear)
{
	if((float)(
			 ((int16)dg_state->L_zh_real - zh) * ((int16)dg_state->L_zh_real - zh) +
			 ((int16)dg_state->L_yh_real - yh) * ((int16)dg_state->L_yh_real - yh) +
			 ((int16)dg_state->L_zx_real - zx) * ((int16)dg_state->L_zx_real - zx) +
			 ((int16)dg_state->L_yx_real - yx) * ((int16)dg_state->L_yx_real - yx) +  
			 ((int16)dg_state->L_zs_real - zs) * ((int16)dg_state->L_zs_real - zs) +
			 ((int16)dg_state->L_ys_real - ys) * ((int16)dg_state->L_ys_real - ys)) /10 <=bear)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}