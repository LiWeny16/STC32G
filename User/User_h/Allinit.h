//���ļ����ڶԳ�ʼ��������������

#ifndef __ALLINIT_H_
#define __ALLINIT_H_

//���ͨ������
#define MOTOR1_P PWMA_CH1P_P60
#define MOTOR1_N PWMA_CH1N_P61
#define MOTOR2_P PWMA_CH3P_P64
#define MOTOR2_N PWMA_CH3N_P65

#define MOTOR1_My_a PWMA_CH1P_P60
#define MOTOR1_My_b PWMA_CH2P_P62
#define MOTOR2_My_a PWMA_CH3P_P64
#define MOTOR2_My_b PWMA_CH4P_P66
//���ͨ������
#define STEERING PWMB_CH1_P74

//����������
#define BUZZER  PWMA_CH2N_P63

//��������������
#define Encoder_L CTIM0_P34
#define Encoder_R CTIM3_P04

#include "common.h"
#include "zf_adc.h"
#include "zf_tim.h"
#include "zf_gpio.h"
#include "zf_pwm.h"
#include "SEEKFREE_WIRELESS.h"
#include "adc.h"
#include "Pid.h"
#include "road.h"
#include "encoder.h"
#include "Variable.h"


void ADC_all_init(void);//���ADCģ���ʼ��
void GPIO_init(void);//IO�ڳ�ʼ��
void PWM_SMB_init(void);//���������ͷ�������PWM��ʼ��
void PIT_init(void);//��ʱ�жϣ����ڵ�м�⣩��ʼ��
void PID_init(void);//PID��ʼ��
void Steering_init(void);//�����ʼ�������У�����������
void Motor_init(void);//�����ʼ��
void FLAG_init(void);//·���жϱ�־λ��ʼ��
void WIRELESS_init(void);//���ߴ��ڳ�ʼ��
void ALL_init(void);//�����ʼ��


#endif