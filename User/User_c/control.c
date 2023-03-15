#include "control.h"

void STEERING_Control(PID_Steering *pid_steering)
{
	pid_steering->PID_STEERING_OUT = constrain_float(pid_steering->PID_STEERING_OUT, PWM_Steering_Min, PWM_Steering_Max);//�����޷�
	pwm_duty(STEERING,(pid_steering->PID_STEERING_OUT+750));//�������
}

void MOTOR_Control(PID_Motor *pid_motor)
{
	pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT+PWM_Motor_L_now;
	pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT+PWM_Motor_R_now;
	
	pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max);//�����޷�
	pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);
	
	//pwm_duty(MOTOR1_P,pid_motor->PID_MOTOR_L_OUT);//��������
	//pwm_duty(MOTOR1_N,pid_motor->PID_MOTOR_L_OUT);
	pwm_duty(PWMA_CH1P_P60,(uint32)0);//��������
	pwm_duty(PWMA_CH2P_P62,pid_motor->PID_MOTOR_L_OUT/4);
	PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT;//����PWM����
	
	pwm_duty(PWMA_CH3P_P64,(uint32)0);//�ҵ������
	pwm_duty(PWMA_CH4P_P66,pid_motor->PID_MOTOR_R_OUT/4);
	//pwm_duty(MOTOR2_P,pid_motor->PID_MOTOR_R_OUT);//�ҵ������
	//pwm_duty(MOTOR2_N,pid_motor->PID_MOTOR_R_OUT);
	PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT;//�ҵ��PWM����
	
}

void Control_All()
{
	//����Ϊ��·�жϲ���
	road = road_judge(&dg_state);//���ݵ�·״�����ص�·�ṹ��
	
	//����Ϊ�����������
	calculate_s(&dg_state,&err_steering);//����ƫ��ֵ��д��ʵ�Σ���allinitʱ����Ѿ���ʼ�ɼ�adc�źţ�
	Pid_Steering_Calculate(&err_steering , &pid_steering);//����pid���ֵ����д��ʵ��
	STEERING_Control(&pid_steering);//��pid�����ʵ�ʶ������
	
	//����Ϊ�����������
	speedout(road,&speed_now,&speed_state);//���ݵ�·������ٶ�״̬�ṹ���е�һ���趨ֵ������ǰ�ٶȽṹ���е�Ŀ��ֵ
	speed_cal(&speed_now);//���ݱ��������ݼ����ٶ�ֵ�������丳����ǰ�ٶȽṹ���еĵ�ǰ�ٶ�ֵ
	calculate_err_m(&speed_now,&err_motor);//���ݱ�������ֵ����ƫ��ֵ��������ƫ��ֵ�ṹ����last��,��������ڵ��ƫ��ֵ�ṹ����
	Pid_Motor_Calculate(&err_motor,&pid_motor);//�ɵ��ƫ��ɱ�������Ӧ���ṹ�������PID�������
	MOTOR_Control(&pid_motor);//��pid�����ʵ�ʵ������
}