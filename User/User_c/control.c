#include "control.h"

void Control_All()
{

	// ����Ϊ�����������
	calculate_s(&dg_state, &err_steering); // ����ƫ��ֵ��д���

	road = road_judge(&timer,&road_flag,&dg_state,&err_steering); // ���ݵ�нṹ���жϵ�·״�����ص�·�ṹ��

	Pid_Steering_Calculate(road, &err_steering, &pid_steering); // ����pid���ֵ����д��ʵ��
	STEERING_Control(road, &pid_steering);						// ��pid�����ʵ�ʶ������,�жϵ�·�ṹ�壬��������Ӧ

	// ����Ϊ�����������
	// �ٶ�ѡ��
	speedout(road, &speed_now, &speed_state);	 // ���ݵ�·������ٶ�״̬�ṹ���е�һ���趨ֵ������ǰ�ٶȽṹ���е�Ŀ��ֵ
	speed_cal(&speed_now);						 // ���ݱ��������ݼ����ٶ�ֵ�������丳����ǰ�ٶȽṹ���еĵ�ǰ�ٶ�ֵ
	calculate_err_m(&speed_now, &err_motor);	 // ���ݱ�������ֵ����ƫ��ֵ��������ƫ��ֵ�ṹ����last��,��������ڵ��ƫ��ֵ�ṹ����
	Pid_Motor_Calculate(&err_motor, &pid_motor); // �ɵ��ƫ��ɱ�������Ӧ���ṹ�������PID�������
	MOTOR_Control(road, &pid_motor);			 // ��pid�����ʵ�ʵ������
}

void STEERING_Control(Road road, PID_Steering *pid_steering)
{
	// temp = pid_steering->PID_STEERING_OUT;
	// pid_steering->PID_STEERING_OUT=(pid_steering->PID_STEERING_OUT+725.0); //֮ǰ������
	float Steer_center_temp = 735.0;
	switch (road)
	{
	case (Straight): // ֱ��
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + Steer_center_temp;
		// temp=(temp+725.0); //֮ǰ����������PIDȨ�غ��ֵ
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));																			// �������
		break;
	case (Curve_Left):
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + Steer_center_temp;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));
		break;
	case (Curve_Right):
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + Steer_center_temp;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));
		break;

	case (Ring_In):
		pwm_duty(STEERING, 788); //��ת����
		break;
	case (Ring_Out):
		pwm_duty(STEERING, 800); //��ת����
		break;
	case (Force_Right):
		pwm_duty(STEERING, 650); //��ת����
		break;
	case (Force_Left):
		pwm_duty(STEERING, 805); //��ת����
		break;
	case (Force_Straight):
		pwm_duty(STEERING, 735); //ֱ����
		break;
	case (Stop): // ͣ��
		pwm_duty(STEERING, Steer_center_temp);
		break;

	default: // ���
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + Steer_center_temp;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));		
		break;
	}
}

void MOTOR_Control(Road road, PID_Motor *pid_motor)
{
	switch (road)
	{
	case (Straight): // ֱ��
		pid_motor->PID_MOTOR_L_OUT = (int)pid_motor->MOTOR_L_OUT_temp + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = (int)pid_motor->MOTOR_R_OUT_temp + PWM_Motor_R_now;
    //PID_MOTOR_L_OUT int32 PWM_Motor_L_now uint16
		pid_motor->PID_MOTOR_L_OUT = constrain_int32(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_int32(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		// pwm_duty(MOTOR1_P,pid_motor->PID_MOTOR_L_OUT);//��������
		// pwm_duty(MOTOR1_N,pid_motor->PID_MOTOR_L_OUT);
		// pwm_duty(PWMA_CH1P_P60,(uint32)0);//��������
		// pwm_duty(PWMA_CH2P_P62,pid_motor->PID_MOTOR_L_OUT/5);
		// 15000 ��Ӧռ�ձ�1400
		pwm_duty(L_Motor_P, 0); // ��������
		pwm_duty(L_Motor_N, 5000+pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(R_Motor_P, 0); // �ҵ������
		pwm_duty(R_Motor_N, 5000+pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;

	case (Curve_Left): //��ת
		pid_motor->PID_MOTOR_L_OUT = (int)pid_motor->MOTOR_L_OUT_temp + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = (int)pid_motor->MOTOR_R_OUT_temp + PWM_Motor_R_now;
    //PID_MOTOR_L_OUT int32 PWM_Motor_L_now uint16
		pid_motor->PID_MOTOR_L_OUT = constrain_int32(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_int32(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);
		pwm_duty(L_Motor_P, 0); // ��������
		pwm_duty(L_Motor_N, 5000+pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(R_Motor_P, 0); // �ҵ������
		pwm_duty(R_Motor_N, 5000+pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;
	case (Curve_Right): //��ת
	pid_motor->PID_MOTOR_L_OUT = (int)pid_motor->MOTOR_L_OUT_temp + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = (int)pid_motor->MOTOR_R_OUT_temp + PWM_Motor_R_now;
    //PID_MOTOR_L_OUT int32 PWM_Motor_L_now uint16
		pid_motor->PID_MOTOR_L_OUT = constrain_int32(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_int32(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);
		pwm_duty(L_Motor_P, 0); // ��������
		pwm_duty(L_Motor_N, 5000+pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(R_Motor_P, 0); // �ҵ������
		pwm_duty(R_Motor_N, 5000+pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;
	case (Stop):					// ͣ��
		pwm_duty(L_Motor_P, 5000); // ��������
		pwm_duty(L_Motor_N, 5000); // ������һ������ֵ����ת

		pwm_duty(R_Motor_P, 5000); // �ҵ������
		pwm_duty(R_Motor_N, 5000);
		break;

	default: // ���
	pid_motor->PID_MOTOR_L_OUT = (int)pid_motor->MOTOR_L_OUT_temp + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = (int)pid_motor->MOTOR_R_OUT_temp + PWM_Motor_R_now;
    //PID_MOTOR_L_OUT int32 PWM_Motor_L_now uint16
		pid_motor->PID_MOTOR_L_OUT = constrain_int32(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_int32(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);
		pwm_duty(L_Motor_P, 0); // ��������
		pwm_duty(L_Motor_N, 5000+pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(R_Motor_P, 0); // �ҵ������
		pwm_duty(R_Motor_N, 5000+pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;
	}
}
