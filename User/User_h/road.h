//���ļ����ڶԵ�·��־λ�ṹ��������Լ�ͬ��c�ļ���Ӧ����������

#ifndef __ROAD_H_
#define __ROAD_H_

#include "common.h"
#include "adc.h"
#include "TempVar.h"
#define Light_Left P67
#define Light_Right P66
typedef enum
{
  Need_judge,
	Stop,
	Curve_Left,
	Curve_Right,
	Ring_Left,
	Ring_Right,
	Ring_In,
	Ring_Out,
	Big_Ring,
	Force_Right,
	Force_Left,
	Force_Straight,
	OutGarage,//�����־
	InGarage,
	Straight,//ֱ��
	       //ʮ�ֽ���
  //***********************
	Cross, 
	No_road,      //���س���ܵ����޷�У����ͣ��
  Out_of_way,   //�Ѿ�����ܵ�
  Lose_left,    //����
  Lose_right,
 
  Three_L,
//  Three_f_cu,
  Three_R,
//  Three_s_cu,
  Finish,//����־
  Stoped,
}Road;
typedef struct // ����ʽPID������ã��ṹ��
{

	int16 Ring_In_Flag;
	int16 Ring_Out_Flag;
	int16 Near_Flag;
	int16 Cross_Flag_Last;
	int16 Cross_Flag;
	int16 InGarage_Flag;
}FLAG;
Road road_judge(FOOT_COUNTER *foot_counter,TIMER *timer,FLAG *road_flag,DG_State *dg_sta ,Err_Steering *err_steering);
int16 isNear(DG_State *dg_state,int16 zh,int16 yh,int16 zx,int16 yx,int16 bear);
int16 isNearPlus(DG_State *dg_state,int16 zh,int16 yh,int16 zx,int16 yx ,int16 zs,int16 ys,int16 bear);
#endif