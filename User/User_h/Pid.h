// ���ļ����ڶ��PID�ṹ��͵��PID�ṹ��������Լ�ͬ��c�ļ��к���������

#ifndef __PID_H_
#define __PID_H_

#include "common.h"
#include "adc.h"
#include "encoder.h"
//
#include "TempVar.h"

//#include "headfile.h"

typedef struct // λ��ʽPID������ã��ṹ��
{

    float p_steering;       // ���ڴ�ű���ϵ��p
    float i_steering;       // ���ڴ�Ż���ϵ��i
    float d_steering;       // ���ڴ��΢��ϵ��d
    float imax;             // �����޷�����
    float imin;             // �����޷�����
	  float STEERING_OUT_temp;
    uint32 PID_STEERING_OUT; // ���ڴ����������������PWMֵ
} PID_Steering;

typedef struct // ����ʽPID������ã��ṹ��
{

    float p_motor;         // ���ڴ�ű���ϵ��p
    float i_motor;         // ���ڴ�Ż���ϵ��i
    float d_motor;         // ���ڴ��΢��ϵ��d
    int32 PID_MOTOR_L_OUT; // ���ڴ��������������������PWM����ֵ
    int32 PID_MOTOR_R_OUT; // ���ڴ��������������������PWM����ֵ
} PID_Motor;

uint32 constrain_uint32(uint32 amt, uint32 low, uint32 high);
float constrain_float(float amt, float low, float high);                             // λ��ʽPID�������޷���PWM����޷���
void Pid_Steering_Calculate(Err_Steering *err_steering, PID_Steering *pid_steering); // �ɶ��ƫ��ɵ�з�Ӧ���ṹ�������PID���ֵ
void Pid_Motor_Calculate(Err_Motor *err_motor, PID_Motor *pid_motor);                // �ɵ��ƫ��ɱ�������Ӧ���ṹ�������PID�������
#endif