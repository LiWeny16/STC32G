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
typedef struct // ����ʽPID������ã��ṹ��
{

	int16 Ring_In_Flag;
	int16 Ring_Out_Flag;
	int16 Near_Flag;

}FLAG;
Road road_judge(FLAG *road_flag,DG_State *dg_sta ,Err_Steering *err_steering);
int16 isNear(DG_State *dg_state,int16 zh,int16 yh,int16 zx,int16 yx,int16 bear);

#endif