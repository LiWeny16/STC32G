// ���ļ�����ʹ�ö��ƫ��ṹ��͵��ƫ��ṹ�������PID���ֵ�͵��PID����ֵ

#include "Pid.h"
float constrain_float(float amt, float low, float high) // �޷��ã�low��highΪ������
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

uint32 constrain_uint32(uint32 amt, uint32 low, uint32 high) // �޷��ã�low��highΪ������
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Pid_Steering_Calculate(Road road, volatile Err_Steering *err_steering, volatile PID_Steering *pid_steering) // ���λ��ʽPID���ֵ
{

	//   constrain_float(err_steering->Errsum, pid_steering->imax, pid_steering->imin);
	// λ��ʽPID�������޷�
	float p_Curve_L;
	float i_Curve_L;
	float d_Curve_L;

	float p_Curve_R;
	float i_Curve_R;
	float d_Curve_R;
	
	p_Curve_L = 25.5;
	i_Curve_L = 0;
	d_Curve_L = 5.5;

	p_Curve_R = 25.5;
	i_Curve_R = 0;
	d_Curve_R = 5.5;
	switch (road)
	{
	case (Straight): // ֱ��
		pid_steering->STEERING_OUT_temp = (float)((pid_steering->p_steering * (err_steering->Err)) + (pid_steering->i_steering * err_steering->Errsum) + (pid_steering->d_steering * err_steering->Errdif));
		break;

	case (Curve_Left): // ���
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_L * err_steering->Err) + (i_Curve_L * err_steering->Errsum) + (d_Curve_L * err_steering->Errdif));
		break;

	case (Curve_Right): // ���
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_R * err_steering->Err) + (i_Curve_R * err_steering->Errsum) + (d_Curve_R * err_steering->Errdif));
		break;

	case (Stop):
		pid_steering->STEERING_OUT_temp = 0;
	default: // ���
		pid_steering->STEERING_OUT_temp = (float)((pid_steering->p_steering * err_steering->Err) + (pid_steering->i_steering * err_steering->Errsum) + (pid_steering->d_steering * err_steering->Errdif));
		break;
	}

	// tempVar =(float)((pid_steering->p_steering * err_steering-> Err) + (pid_steering->i_steering * err_steering-> Errsum) + (pid_steering->d_steering * err_steering-> Errdif));
	// pid_steering->PID_STEERING_OUT = (f)(((pid_steering->p_steering * err_steering-> Err*(-1.0)) + (pid_steering->i_steering * err_steering-> Errsum*0) + (pid_steering->d_steering * err_steering-> Errdif)*0));
	//	temp =   pid_steering->PID_STEERING_OUT
	// λ��ʽPID�������
}

void Pid_Motor_Calculate(Err_Motor *err_motor, PID_Motor *pid_motor) // �������ʽPID�������
{
	pid_motor->PID_MOTOR_L_OUT = (pid_motor->p_motor * err_motor->err_derivative_L_m) + (pid_motor->i_motor * err_motor->err_L_m) + (pid_motor->d_motor * err_motor->err_derivative2_L_m);
	pid_motor->PID_MOTOR_R_OUT = (pid_motor->p_motor * err_motor->err_derivative_R_m) + (pid_motor->i_motor * err_motor->err_R_m) + (pid_motor->d_motor * err_motor->err_derivative2_R_m);
}