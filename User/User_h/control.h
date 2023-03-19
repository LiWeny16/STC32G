// ���ļ����ڶԳ�ʼ��������������

#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "common.h"
#include "zf_pwm.h"   //ʹ������pwm����ռ�ձȺ���
#include "Allinit.h"  //ʹ�����е��ͨ������
#include "Variable.h" //ʹ�����е��������ȫ�ֱ���
#include "Pid.h"      //ʹ�����е��޷�����
#include "adc.h"      //ʹ��������ؼ��㺯��
#include "encoder.h"  //ʹ��������ؼ��㺯��
#include "road.h"     //ʹ�õ�·�жϽṹ��͵�·�жϺ���

void STEERING_Control(Road road,PID_Steering *pid_steering); // ��������������PID��ʵ�������
void MOTOR_Control(Road road,PID_Motor *pid_motor);          // ��������������PID��ʵ�������
void Control_All();                                // �ܿ��Ʒ�װ�����������->ƫ��->���PID->ʵ������,������->�ٶ�->ƫ��->���PID->ʵ������
#endif