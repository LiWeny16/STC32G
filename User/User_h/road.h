//���ļ����ڶԵ�·��־λ�ṹ��������Լ�ͬ��c�ļ���Ӧ����������

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
	
  No_road,      //���س���ܵ����޷�У����ͣ��
  Out_of_way,   //�Ѿ�����ܵ�
  Lose_left,    //����
  Lose_right,
  Cross,        //ʮ�ֽ���
  Straight,//ֱ��
  Curve, //���
  LeftRing,//��Բ��
  RightRing,//����Բ��
  LeftOutRing,//����Բ��
  RightOutRing,//��Բ��
  Three_L,
//  Three_f_cu,
  Three_R,
//  Three_s_cu,
  Finish,//����־
  OutGarage,//�����־
  InGarage,
  Stoped,
}Road;

Road road_judge(DG_State* dg_sta ,Err_Steering *err_steering);


#endif