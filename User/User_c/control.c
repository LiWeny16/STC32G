#include "control.h"

void Control_All()
{

	// ����Ϊ�����������
	calculate_s(&dg_state, &err_steering); // ����ƫ��ֵ��д���

	road = road_judge(&dg_state,&err_steering); // ���ݵ�нṹ���жϵ�·״�����ص�·�ṹ��

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
	switch (road)
	{
	case (Straight): // ֱ��
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		// temp=(temp+725.0); //֮ǰ����������PIDȨ�غ��ֵ
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));																			// �������
		break;
	case (Curve_Left):
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));
		break;
	case (Curve_Right):
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
		pid_steering->PID_STEERING_OUT = constrain_uint32((uint32)pid_steering->STEERING_OUT_temp, PWM_Steering_Min, PWM_Steering_Max); // �����޷�
		pwm_duty(STEERING, (pid_steering->PID_STEERING_OUT));
		break;

	case (Ring_In):
		pwm_duty(STEERING, 800); //��ת����
		break;
	case (Ring_Out):
		pwm_duty(STEERING, 800); //��ת����
		break;
	case (Stop): // ͣ��
		pwm_duty(STEERING, 725);
		break;

	default: // ���
		pid_steering->STEERING_OUT_temp = pid_steering->STEERING_OUT_temp + 725.0;
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
		pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		// pwm_duty(MOTOR1_P,pid_motor->PID_MOTOR_L_OUT);//��������
		// pwm_duty(MOTOR1_N,pid_motor->PID_MOTOR_L_OUT);
		// pwm_duty(PWMA_CH1P_P60,(uint32)0);//��������
		// pwm_duty(PWMA_CH2P_P62,pid_motor->PID_MOTOR_L_OUT/5);
		// 15000 ��Ӧռ�ձ�1400
		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // ��������
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // �ҵ������
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;

	case (Curve_Left): //��ת
				pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // ��������
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // �ҵ������
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;
	case (Curve_Right): //��ת
				pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // ��������
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // �ҵ������
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;
	case (Stop):					// ͣ��
		pwm_duty(PWMA_CH1P_P60, 0); // ��������
		pwm_duty(PWMA_CH2P_P62, 0); // ������һ������ֵ����ת

		pwm_duty(PWMA_CH3P_P64, 0); // �ҵ������
		pwm_duty(PWMA_CH4P_P66, 0);
		break;

	default: // ���
		pid_motor->PID_MOTOR_L_OUT = pid_motor->PID_MOTOR_L_OUT + PWM_Motor_L_now;
		pid_motor->PID_MOTOR_R_OUT = pid_motor->PID_MOTOR_R_OUT + PWM_Motor_R_now;

		pid_motor->PID_MOTOR_L_OUT = constrain_float(pid_motor->PID_MOTOR_L_OUT, PWM_Motor_Min, PWM_Motor_Max); // �����޷�
		pid_motor->PID_MOTOR_R_OUT = constrain_float(pid_motor->PID_MOTOR_R_OUT, PWM_Motor_Min, PWM_Motor_Max);

		pwm_duty(PWMA_CH1P_P60, pid_motor->PID_MOTOR_L_OUT); // ��������
		pwm_duty(PWMA_CH2P_P62, pid_motor->PID_MOTOR_L_OUT); // ������һ������ֵ����ת

		pwm_duty(PWMA_CH3P_P64, pid_motor->PID_MOTOR_R_OUT); // �ҵ������
		pwm_duty(PWMA_CH4P_P66, pid_motor->PID_MOTOR_R_OUT);
		PWM_Motor_L_now = pid_motor->PID_MOTOR_L_OUT; // ����PWM����
		PWM_Motor_R_now = pid_motor->PID_MOTOR_R_OUT; // �ҵ��PWM����
		break;
	}
}
