//���ļ����ڶԸ�����Ҫ�õ��ı������и�ֵ


#include "Variable.h"

volatile Err_Steering err_steering;//���ƫ��ṹ�壨ʵ�Σ�
volatile DG_State dg_state;//���״̬�ṹ�壨ʵ�Σ�
volatile PID_Steering pid_steering;//���PID�ṹ�壨ʵ�Σ�
volatile uint16  PWM_Steering_now;//��ǰ���ռ�ձ�
volatile uint16  PWM_Steering_Max;//������ռ�ձ�
volatile uint16  PWM_Steering_Min;//�����Сռ�ձ�



volatile SPEED_state speed_state;//����ٶ�״̬�ṹ�壬���ڴ�Ų�ͬ·��ʱ��Ŀ���ٶȣ�ʵ�Σ�
volatile SPEED_now speed_now;//�����ǰ�ٶȽṹ�壬���ڴ��������Ŀ���ٶȺ͵�ǰ�ٶȣ�ʵ�Σ�
volatile Err_Motor err_motor;//���ƫ��ṹ�壨ʵ�Σ�
volatile PID_Motor pid_motor;//���PID�ṹ�壨ʵ�Σ�
volatile uint16  PWM_Motor_Max;//������ռ�ձ�
volatile uint16  PWM_Motor_Min;//�����Сռ�ձ�
volatile uint16  PWM_Motor_L_now;//��ǰ����ռ�ձ�
volatile uint16  PWM_Motor_R_now;//��ǰ�ҵ��ռ�ձ�


volatile Road road;//��·�жϽṹ��