#include "control.h"

void STEERING_Control(PID_Steering *pid_steering)
{
	//temp = pid_steering->PID_STEERING_OUT;
	//pid_steering->PID_STEERING_OUT=(pid_steering->PID_STEERING_OUT+725.0); //之前有正负
	pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
	//temp=(temp+725.0); //之前有正负

	pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max);//驱动限幅
	pwm_duty(STEERING,(pid_steering->PID_STEERING_OUT));//舵机驱动
}

void MOTOR_Control(PID_Motor *pid_motor)
{
	pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT+PWM_Motor_L_now;
	pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT+PWM_Motor_R_now;
	
	pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max);//驱动限幅
	pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);
	
	//pwm_duty(MOTOR1_P,pid_motor->PID_MOTOR_L_OUT);//左电机驱动
	//pwm_duty(MOTOR1_N,pid_motor->PID_MOTOR_L_OUT);
	pwm_duty(PWMA_CH1P_P60,(uint32)0);//左电机驱动
	pwm_duty(PWMA_CH2P_P62,pid_motor->PID_MOTOR_L_OUT/5);
	PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT;//左电机PWM更新
	
	pwm_duty(PWMA_CH3P_P64,(uint32)0);//右电机驱动
	pwm_duty(PWMA_CH4P_P66,pid_motor->PID_MOTOR_R_OUT/5);
	//pwm_duty(MOTOR2_P,pid_motor->PID_MOTOR_R_OUT);//右电机驱动
	//pwm_duty(MOTOR2_N,pid_motor->PID_MOTOR_R_OUT);
	PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT;//右电机PWM更新
	
}

void Control_All()
{
	//以下为道路判断部分
	road = road_judge(&dg_state);//根据道路状况返回道路结构体
	
	//以下为舵机驱动部分
	calculate_s(&dg_state,&err_steering);//计算偏差值，写入实参（在allinit时电感已经开始采集adc信号）
	Pid_Steering_Calculate(&err_steering , &pid_steering);//计算pid输出值，并写入实参
	STEERING_Control(&pid_steering);//从pid输出到实际舵机驱动
	
	//以下为电机驱动部分
	speedout(road,&speed_now,&speed_state);//根据道路情况将速度状态结构体中的一组设定值赋给当前速度结构体中的目标值
	speed_cal(&speed_now);//根据编码器数据计算速度值，并将其赋给当前速度结构体中的当前速度值
	calculate_err_m(&speed_now,&err_motor);//根据编码器数值计算偏差值，并更新偏差值结构体中last量,结果保存在电机偏差值结构体中
	Pid_Motor_Calculate(&err_motor,&pid_motor);//由电机偏差（由编码器反应）结构体计算电机PID输出增量
	MOTOR_Control(&pid_motor);//从pid输出到实际电机驱动
}