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

int32 constrain_int32(int32 amt, int32 low, int32 high) // 限幅用，low和high为上下限
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

void Pid_Steering_Calculate(Road road, volatile Err_Steering *err_steering, volatile PID_Steering *pid_steering) // 舵机位置式PID输出值
{

	//   constrain_float(err_steering->Errsum, pid_steering->imax, pid_steering->imin);
	// 位置式PID积分项限幅
	float p_Str;
	float i_Str;
	float d_Str;
	
	float p_Curve_L;
	float i_Curve_L;
	float d_Curve_L;

	float p_Curve_R;
	float i_Curve_R;
	float d_Curve_R;
	
	float p_Ring_R;
	float i_Ring_R;
	float d_Ring_R;
	
	float p_Ring_L;
	float i_Ring_L;
	float d_Ring_L;
	// 直线
	p_Str= 13.3;
	i_Str=1.0;
	d_Str=0.378;
	
	// 曲线左
	p_Curve_L = 20.5;
	i_Curve_L = 0.8;
	d_Curve_L = 1.378;
	// 曲线右
	p_Curve_R = 19.5;
	i_Curve_R = 0.8;
	d_Curve_R = 1.378;
	
	// 环左
	p_Ring_L = 22.8;
	i_Ring_L = 0.10;
	d_Ring_L = 1.82;
	// 环右
	p_Ring_R = 30.8;
	i_Ring_R = 0.10;
	d_Ring_R = 1.82;
	switch (road)
	{
	case (Straight): // 直道
		err_steering->Err=(0.4151 * (err_steering->Err_x)) + (3.4868 * (err_steering->Err_h));
		err_steering->Errsum += err_steering->Err;
		err_steering->Errsum = err_steering->Errsum>2?2:err_steering->Errsum;
		err_steering->Errsum = err_steering->Errsum<-2?-2:err_steering->Errsum;
		err_steering->Errdif = err_steering->Err - err_steering->Err_last;
		err_steering->Err_last = err_steering->Err;
		pid_steering->STEERING_OUT_temp = (float)((p_Str * err_steering->Err) + (i_Str * err_steering->Errsum) + (d_Str * err_steering->Errdif));
		break;

	case (Curve_Left): // 弯道
		err_steering->Err=(0.9151 * (err_steering->Err_x)) + (4.5868 * (err_steering->Err_h));
		err_steering->Errsum += err_steering->Err;
		err_steering->Errsum = err_steering->Errsum>2?2:err_steering->Errsum;
		err_steering->Errsum = err_steering->Errsum<-2?-2:err_steering->Errsum;
		err_steering->Errdif = err_steering->Err - err_steering->Err_last;
		err_steering->Err_last = err_steering->Err;
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_L * err_steering->Err) + (i_Curve_L * err_steering->Errsum) + (d_Curve_L * err_steering->Errdif));
		break;

	case (Curve_Right): // 弯道
		err_steering->Err=(0.9151 * (err_steering->Err_x)) + (4.5868 * (err_steering->Err_h));
		err_steering->Errsum += err_steering->Err;
		err_steering->Errsum = err_steering->Errsum>2?2:err_steering->Errsum;
		err_steering->Errsum = err_steering->Errsum<-2?-2:err_steering->Errsum;
		err_steering->Errdif = err_steering->Err - err_steering->Err_last;
		err_steering->Err_last = err_steering->Err;
		pid_steering->STEERING_OUT_temp = (float)((p_Curve_R * err_steering->Err) + (i_Curve_R * err_steering->Errsum) + (d_Curve_R * err_steering->Errdif));
		break;
	case (Ring_Left):
		err_steering->Err=(1.2551 * (err_steering->Err_x)) + (4.5868 * (err_steering->Err_h));
		err_steering->Errsum += err_steering->Err;
		err_steering->Errsum = err_steering->Errsum>2?2:err_steering->Errsum;
		err_steering->Errsum = err_steering->Errsum<-2?-2:err_steering->Errsum;
		err_steering->Errdif = err_steering->Err - err_steering->Err_last;
		err_steering->Err_last = err_steering->Err;
		pid_steering->STEERING_OUT_temp = (float)((p_Ring_L * err_steering->Err) + (i_Ring_L * err_steering->Errsum) + (d_Ring_L * err_steering->Errdif));
		break;
	case (Ring_Right):
		err_steering->Err=(1.2551 * (err_steering->Err_x)) + (4.5868 * (err_steering->Err_h));
		err_steering->Errsum += err_steering->Err;
		err_steering->Errsum = err_steering->Errsum>2?2:err_steering->Errsum;
		err_steering->Errsum = err_steering->Errsum<-2?-2:err_steering->Errsum;
		err_steering->Errdif = err_steering->Err - err_steering->Err_last;
		err_steering->Err_last = err_steering->Err;
		pid_steering->STEERING_OUT_temp = (float)((p_Ring_R * err_steering->Err) + (i_Ring_R * err_steering->Errsum) + (d_Ring_R * err_steering->Errdif));
		break;
	
	case (Stop):
		pid_steering->STEERING_OUT_temp = 0.0;
		break;
	default: // 别的
		err_steering->Err=(0.4151 * (err_steering->Err_x)) + (3.4868 * (err_steering->Err_h));
		err_steering->Errsum += err_steering->Err;
		err_steering->Errsum = err_steering->Errsum>2?2:err_steering->Errsum;
		err_steering->Errsum = err_steering->Errsum<-2?-2:err_steering->Errsum;
		err_steering->Errdif = err_steering->Err - err_steering->Err_last;
		err_steering->Err_last = err_steering->Err;
		pid_steering->STEERING_OUT_temp = (float)((p_Str * err_steering->Err) + (i_Str * err_steering->Errsum) + (d_Str * err_steering->Errdif));
		break;
	}
	//err_steering->Errsum += err_steering->Err;
  //err_steering->Errdif = err_steering->Err - err_steering->Err_last;
  //err_steering->Err_last = err_steering->Err;
	// tempVar =(float)((pid_steering->p_steering * err_steering-> Err) + (pid_steering->i_steering * err_steering-> Errsum) + (pid_steering->d_steering * err_steering-> Errdif));
	// pid_steering->PID_STEERING_OUT = (f)(((pid_steering->p_steering * err_steering-> Err*(-1.0)) + (pid_steering->i_steering * err_steering-> Errsum*0) + (pid_steering->d_steering * err_steering-> Errdif)*0));
	//	temp =   pid_steering->PID_STEERING_OUT
	// 位置式PID输出计算
}

void Pid_Motor_Calculate(Err_Motor *err_motor, PID_Motor *pid_motor) // 电机增量式PID输出增量
{
	pid_motor->MOTOR_L_OUT_temp= (pid_motor->p_motor * err_motor->err_derivative_L_m) + (pid_motor->i_motor * err_motor->err_L_m) + (pid_motor->d_motor * err_motor->err_derivative2_L_m);
	pid_motor->MOTOR_R_OUT_temp = (pid_motor->p_motor * err_motor->err_derivative_R_m) + (pid_motor->i_motor * err_motor->err_R_m) + (pid_motor->d_motor * err_motor->err_derivative2_R_m);

}