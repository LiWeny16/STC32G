// 该文件用于使用舵机偏差结构体和电机偏差结构体计算舵机PID输出值和电机PID增量值

#include "Pid.h"
float constrain_float(float amt, float low, float high) // 限幅用，low和high为上下限
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

uint32 constrain_uint32(uint32 amt, uint32 low, uint32 high) // 限幅用，low和high为上下限
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Pid_Steering_Calculate(Road road, volatile Err_Steering *err_steering, volatile PID_Steering *pid_steering) // 舵机位置式PID输出值
{

	//   constrain_float(err_steering->Errsum, pid_steering->imax, pid_steering->imin);
	// 位置式PID积分项限幅
	float p_Curve_L;
	float i_Curve_L;
	float d_Curve_L;

	float p_Curve_R;
	float i_Curve_R;
	float d_Curve_R;
	
	p_Curve_L = 23.5;
	i_Curve_L = 0;
	d_Curve_L = 5.5;

	p_Curve_R = 23.5;
	i_Curve_R = 0;
	d_Curve_R = 5.5;
	switch (road)
	{
	case (Straight): // 直道
		err_steering->Err=(0.9151 * (err_steering->Err_x)) + (3.4868 * (err_steering->Err_h));
		pid_steering->STEERING_OUT_temp = (float)((pid_steering->p_steering * (err_steering->Err)) + (pid_steering->i_steering * err_steering->Errsum) + (pid_steering->d_steering * err_steering->Errdif));
		break;

	case (Curve_Left): // 弯道
		err_steering->Err=(2.4151 * (err_steering->Err_x)) + (3.9868 * (err_steering->Err_h));
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_L * err_steering->Err) + (i_Curve_L * err_steering->Errsum) + (d_Curve_L * err_steering->Errdif));
		break;

	case (Curve_Right): // 弯道
		err_steering->Err=(2.4151 * (err_steering->Err_x)) + (3.9868 * (err_steering->Err_h));
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_R * err_steering->Err) + (i_Curve_R * err_steering->Errsum) + (d_Curve_R * err_steering->Errdif));
		break;
	case (Big_Ring):
		err_steering->Err=(2.4151 * (err_steering->Err_x)) + (3.9868 * (err_steering->Err_h));
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_R * err_steering->Err) + (i_Curve_R * err_steering->Errsum) + (d_Curve_R * err_steering->Errdif));
		break;
	
	case (Stop):
		pid_steering->STEERING_OUT_temp = 0;
	default: // 别的
		pid_steering->STEERING_OUT_temp = (float)((pid_steering->p_steering * err_steering->Err) + (pid_steering->i_steering * err_steering->Errsum) + (pid_steering->d_steering * err_steering->Errdif));
		break;
	}

	// tempVar =(float)((pid_steering->p_steering * err_steering-> Err) + (pid_steering->i_steering * err_steering-> Errsum) + (pid_steering->d_steering * err_steering-> Errdif));
	// pid_steering->PID_STEERING_OUT = (f)(((pid_steering->p_steering * err_steering-> Err*(-1.0)) + (pid_steering->i_steering * err_steering-> Errsum*0) + (pid_steering->d_steering * err_steering-> Errdif)*0));
	//	temp =   pid_steering->PID_STEERING_OUT
	// 位置式PID输出计算
}

void Pid_Motor_Calculate(Err_Motor *err_motor, PID_Motor *pid_motor) // 电机增量式PID输出增量
{
	pid_motor->PID_MOTOR_L_OUT = (pid_motor->p_motor * err_motor->err_derivative_L_m) + (pid_motor->i_motor * err_motor->err_L_m) + (pid_motor->d_motor * err_motor->err_derivative2_L_m);
	pid_motor->PID_MOTOR_R_OUT = (pid_motor->p_motor * err_motor->err_derivative_R_m) + (pid_motor->i_motor * err_motor->err_R_m) + (pid_motor->d_motor * err_motor->err_derivative2_R_m);
}