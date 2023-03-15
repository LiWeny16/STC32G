//���ļ�����ʹ�ö��ƫ��ṹ��͵��ƫ��ṹ�������PID���ֵ�͵��PID����ֵ

#include "Pid.h"
float constrain_float(float amt, float low, float high)//�޷��ã�low��highΪ������
{
	 return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void Pid_Steering_Calculate(volatile Err_Steering* err_steering , volatile PID_Steering* pid_steering)//���λ��ʽPID���ֵ
{

    constrain_float(err_steering->Errsum, pid_steering->imax, pid_steering->imin);
//λ��ʽPID�������޷�
	
    pid_steering->PID_STEERING_OUT = (pid_steering->p_steering * err_steering-> Err) + (pid_steering->i_steering * err_steering-> Errsum) + (pid_steering->d_steering * err_steering-> Errdif);
//λ��ʽPID�������
}

void Pid_Motor_Calculate(Err_Motor* err_motor,PID_Motor* pid_motor)//�������ʽPID�������
{
	pid_motor->PID_MOTOR_L_OUT = (pid_motor->p_motor * err_motor->err_derivative_L_m) + (pid_motor->i_motor * err_motor->err_L_m) + (pid_motor->d_motor * err_motor->err_derivative2_L_m);
	pid_motor->PID_MOTOR_R_OUT = (pid_motor->p_motor * err_motor->err_derivative_R_m) + (pid_motor->i_motor * err_motor->err_R_m) + (pid_motor->d_motor * err_motor->err_derivative2_R_m);
	
}