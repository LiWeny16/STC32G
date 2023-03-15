//���ļ������ٶ�״̬�ṹ�弴���ƫ��ֵ�ṹ��������Լ�ͬ��c�ļ�����Ӧ����������

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "common.h"
#include "zf_tim.h"//���ļ���Ҫ���ڶ�ȡ����������
#include "road.h"
#include "headfile.h"

#define DIR_L P35
#define DIR_R P36
#define Encoder_L CTIM0_P34
#define Encoder_R CTIM3_P04
#define CONTROL_T 0.02  //��ֵ�豣��Ϊ������
//�������ڵĵ������������ö�ʱ�����ж����ڣ���λΪs���������Լ����ٶȣ����������20ms�����ֵΪ0.02
#define DECO 11587
//С����1mʱ��������������
#define MFBL 10000
//����PID���Ƶľ���������������ֵ0�ĸ���ȡ����Ϊ10000�����ٶ���������������Ϊǧ


typedef struct//�ٶ�״̬�ṹ�壨���ٶȱ�ʾ��
{
	      //   ��Ϊ�趨��Ŀ���ٶ�ֵ
        uint32 Outgar_speed_L_ai;//������
        uint32 Outgar_speed_R_ai;//������
	
        uint32 Strai_speed_L_ai;//ֱ����
        uint32 Strai_speed_R_ai;//ֱ����
	
        uint32 Cur_speed_L_ai;//�����
        uint32 Cur_speed_R_ai;//�����
	
        uint32 Cross_speed_L_ai;//ʮ����
        uint32 Cross_speed_R_ai;//ʮ����
	
        uint32 Rampin_speed_L_ai;//���µ���
        uint32 Rampin_speed_R_ai;//���µ���
	
        uint32 Ramp_speed_L_ai;//�µ���
	      uint32 Ramp_speed_R_ai;//�µ���
	
        uint32 Ring_speed_L_ai;//Բ���ڲ���
	      uint32 Ring_speed_R_ai;//Բ���ڲ���
				
        uint32 Ringin_speed_L_ai;//��Բ����
				uint32 Ringin_speed_R_ai;//��Բ����
				
        uint32 Ringout_speed_L_ai;//��Բ����
				uint32 Ringout_speed_R_ai;//��Բ����
				
        uint32 Three_speed_L_ai;//������      
				uint32 Three_speed_R_ai;//������
				
        uint32 Threein_speed_L_ai;//��������
				uint32 Threein_speed_R_ai;//��������
        
} SPEED_state;

typedef struct//����ʽPID������ã�ƫ��ֵ�ṹ��
{

    int32 err_L_m;//���ݵ�ǰ��������������������ƫ��ֵ
    int32 err_last_L_m;//���ڴ����һ�μ����������ƫ��ֵ
    int32 err_past_L_m;//���ڴ������һ�μ����������ƫ��ֵ
	  int32 err_derivative_L_m;//���ڴ�����ֱ���ƫ�����ϴ�ƫ��֮��
	  int32 err_derivative2_L_m;//���ڴ�������ϴ�ƫ�������ϴ�ƫ��֮��

	  int32 err_R_m;//���ݵ�ǰ��������������������ƫ��ֵ
    int32 err_last_R_m;//���ڴ����һ�μ����������ƫ��ֵ
    int32 err_past_R_m;//���ڴ������һ�μ����������ƫ��ֵ
		int32 err_derivative_R_m;//���ڴ�����ֱ���ƫ�����ϴ�ƫ��֮��
	  int32 err_derivative2_R_m;//���ڴ�������ϴ�ƫ�������ϴ�ƫ��֮��
	
}Err_Motor;

typedef struct//��ǰ�ٶȽṹ�壨������ǰ״̬�³�������״̬��ֵ�����ٶȱ�ʾ��
{
	uint32 speed_L;//���ֵ�ǰ�ٶ�ֵ
	uint32 speed_L_ai;//����Ŀ��ֵ
	uint32 speed_R;//���ֵ�ǰ�ٶ�ֵ
	uint32 speed_R_ai;//����Ŀ��ֵ
}SPEED_now; 


void speedout(Road road,SPEED_now* speed_now,SPEED_state* speed_state);//���ݵ�·������ٶ�״̬�ṹ���е�һ���趨ֵ������ǰ�ٶȽṹ���е�Ŀ��ֵ
void speed_cal(SPEED_now* speed_now);//���ݱ��������ݼ����ٶ�ֵ�������丳����ǰ�ٶȽṹ���еĵ�ǰ�ٶ�ֵ
void calculate_err_m(SPEED_now* speed_now,Err_Motor* err_Mot);//���ݱ�������ֵ����ƫ��ֵ��������ƫ��ֵ�ṹ����last��,��������ڵ��ƫ��ֵ�ṹ����
#endif