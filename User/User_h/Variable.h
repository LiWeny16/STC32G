//���ļ���Ҫ���ڽ���Ҫ�ı�������Ϊȫ�ֱ�������ṹ��ȣ�
//��ע���ṹ�嶨�塢�ṹ������Ϊȫ�ֱ������ṹ��ʹ�����������нṹ������Ϊȫ�ֱ������ṹ��ʹ���������ֱ�������ṹ�嶨�岿�ֵ�ͷ�ļ�������ļ���Err_Steering��adc.h�ж��壬��Variable.h������Ϊȫ�ֱ�������Pid.h��ʹ�ã���Variable.h��Pid.h�����adc.h��

#ifndef __VARIABLE_H_
#define __VARIABLE_H_

#include "common.h"
#include "adc.h"
#include "encoder.h"
#include "Pid.h"


/**************��������******/

extern volatile Err_Steering err_steering;//���ƫ��ṹ�壨ʵ�Σ�
extern volatile DG_State dg_state;//���״̬�ṹ�壨ʵ�Σ�
extern volatile PID_Steering pid_steering;//���PID�ṹ�壨ʵ�Σ�
extern volatile uint16  PWM_Steering_now;//��ǰ���ռ�ձ�
extern volatile uint16  PWM_Steering_Max;//������ռ�ձ�
extern volatile uint16  PWM_Steering_Min;//�����Сռ�ձ�



extern volatile SPEED_state speed_state;//����ٶ�״̬�ṹ�壬���ڴ�Ų�ͬ·��ʱ��Ŀ���ٶȣ�ʵ�Σ�
extern volatile SPEED_now speed_now;//�����ǰ�ٶȽṹ�壬���ڴ��������Ŀ���ٶȺ͵�ǰ�ٶȣ�ʵ�Σ�
extern volatile Err_Motor err_motor;//���ƫ��ṹ�壨ʵ�Σ�
extern volatile PID_Motor pid_motor;//���PID�ṹ�壨ʵ�Σ�
extern volatile uint16  PWM_Motor_Max;//������ռ�ձ�
extern volatile uint16  PWM_Motor_Min;//�����Сռ�ձ�
extern volatile uint16  PWM_Motor_L_now;//��ǰ����ռ�ձ�
extern volatile uint16  PWM_Motor_R_now;//��ǰ�ҵ��ռ�ձ�


extern volatile Road road;//��·�жϽṹ��

#endif