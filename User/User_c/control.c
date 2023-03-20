#include "control.h"

void Control_All()
{

	// 以下为舵机驱动部分
	calculate_s(&dg_state, &err_steering); // 计算偏差值，写电感

	road = road_judge(&dg_state,&err_steering); // 根据电感结构体判断道路状况返回道路结构体

	Pid_Steering_Calculate(road, &err_steering, &pid_steering); // 计算pid输出值，并写入实参
	STEERING_Control(road, &pid_steering);						// 从pid输出到实际舵机驱动,判断道路结构体，并做出响应

	// 以下为电机驱动部分
	// 速度选择
	speedout(road, &speed_now, &speed_state);	 // 根据道路情况将速度状态结构体中的一组设定值赋给当前速度结构体中的目标值
	speed_cal(&speed_now);						 // 根据编码器数据计算速度值，并将其赋给当前速度结构体中的当前速度值
	calculate_err_m(&speed_now, &err_motor);	 // 根据编码器数值计算偏差值，并更新偏差值结构体中last量,结果保存在电机偏差值结构体中
	Pid_Motor_Calculate(&err_motor, &pid_motor); // 由电机偏差（由编码器反应）结构体计算电机PID输出增量
	MOTOR_Control(road, &pid_motor);			 // 从pid输出到实际电机驱动
}

void STEERING_Control(Road road, PID_Steering *pid_steering)
{
	// temp = pid_steering->PID_STEERING_OUT;
	// pid_steering->PID_STEERING_OUT=(pid_steering->PID_STEERING_OUT+725.0); //之前有正负
	switch (road)
	{
	case (Straight): // 直道
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		// temp=(temp+725.0); //之前有正负，是PID权重后的值
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // 驱动限幅
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));																			// 舵机驱动
		break;
	case (Curve_Left):
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // 驱动限幅
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));
		break;
	case (Curve_Right):
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // 驱动限幅
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));
		break;

	case (Ring_In):
		pwm_duty(STEERING, 800); //左转打死
		break;
	case (Ring_Out):
		pwm_duty(STEERING, 800); //左转打死
		break;
	case (Stop): // 停下
		pwm_duty(STEERING, 725);
		break;

	default: // 别的
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // 驱动限幅
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));		
		break;
	}
}

void MOTOR_Control(Road road, PID_Motor *pid_motor)
{
	switch (road)
	{
	case (Straight): // 直道
		pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // 驱动限幅
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		// pwm_duty(MOTOR1_P,pid_motor->PID_MOTOR_L_OUT);//左电机驱动
		// pwm_duty(MOTOR1_N,pid_motor->PID_MOTOR_L_OUT);
		// pwm_duty(PWMA_CH1P_P60,(uint32)0);//左电机驱动
		// pwm_duty(PWMA_CH2P_P62,pid_motor->PID_MOTOR_L_OUT/5);
		// 15000 对应占空比1400
		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // 左电机驱动
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // 单独它一个有数值，反转

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // 右电机驱动
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // 左电机PWM更新
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // 右电机PWM更新
		break;

	case (Curve_Left): //左转
				pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // 驱动限幅
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // 左电机驱动
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // 单独它一个有数值，反转

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // 右电机驱动
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // 左电机PWM更新
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // 右电机PWM更新
		break;
	case (Curve_Right): //右转
				pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // 驱动限幅
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // 左电机驱动
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // 单独它一个有数值，反转

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // 右电机驱动
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // 左电机PWM更新
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // 右电机PWM更新
		break;
	case (Stop):					// 停下
		pwm_duty(PWMA_CH1P_P60, 0); // 左电机驱动
		pwm_duty(PWMA_CH2P_P62, 0); // 单独它一个有数值，反转

		pwm_duty(PWMA_CH3P_P64, 0); // 右电机驱动
		pwm_duty(PWMA_CH4P_P66, 0);
		break;

	default: // 别的
		pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // 驱动限幅
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // 左电机驱动
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // 单独它一个有数值，反转

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // 右电机驱动
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // 左电机PWM更新
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // 右电机PWM更新
		break;
	}
}
