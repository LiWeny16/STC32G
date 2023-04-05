// ���ļ����ڽ������ֳ�ʼ��������������Ϊһ������Allinit����

#include "Allinit.h"
// 1.���ADCģ���ʼ����������ʼ����IO��ΪADC�����ڣ����������Ƶ��ΪSYSclk��һ�룩
void ADC_all_init(void)
{
	adc_init(ADC_P00, ADC_SYSclk_DIV_2);
	adc_init(ADC_P01, ADC_SYSclk_DIV_2);
	adc_init(ADC_P05, ADC_SYSclk_DIV_2);
	adc_init(ADC_P06, ADC_SYSclk_DIV_2);
	// adc_init(ADC_P11, ADC_SYSclk_DIV_2);
	// adc_init(ADC_P10, ADC_SYSclk_DIV_2);

	err_steering.Err_x = 0; // ���ƫ��ṹ�壨ʵ�Σ�
	err_steering.Err_h = 0;
	err_steering.Err_s = 0;
	err_steering.Err = 0;
	err_steering.Err_last = 0;
	err_steering.Err_x_last = 0;
	err_steering.Err_h_last = 0;
	err_steering.Errsum = 0;
	err_steering.Errdif = 0;

	dg_state.L_zx_real = 0; // ���״̬�ṹ�壨ʵ�Σ�
	dg_state.L_zh_real = 0;
//	dg_state.L_zs_real = 0;
	//dg_state.L_ys_real = 0;
	dg_state.L_yh_real = 0;
	dg_state.L_yx_real = 0;

	// dg_state.L_zx_max = 33; // ��⵽�����ֵ
	// dg_state.L_zh_max = 2183;
	// dg_state.L_zs_max = 0;
	// dg_state.L_ys_max = 0;
	// dg_state.L_yh_max = 1831;
	// dg_state.L_yx_max = 1700;
	dg_state.L_zx_max = 4096.0; // ��⵽�����ֵ
	dg_state.L_zh_max = 4096.0;
//	dg_state.L_zs_max = 0;
//	dg_state.L_ys_max = 0;
	dg_state.L_yh_max = 4096.0;
	dg_state.L_yx_max = 4096.0;

	dg_state.L_zx_once = 0; // һ�ι�һ����
	dg_state.L_zh_once = 0;
	dg_state.L_zs_once = 0;
	dg_state.L_ys_once = 0;
	dg_state.L_yh_once = 0;
	dg_state.L_yx_once = 0;

	dg_state.L_zx_twice = 0; // ���ι�һ����
	dg_state.L_zh_twice = 0;
	dg_state.L_zs_twice = 0;
	dg_state.L_ys_twice = 0;
	dg_state.L_yh_twice = 0;
	dg_state.L_yx_twice = 0;

	/*
	adc_init(ADC_P14,ADC_SYSclk_DIV_2);
	adc_init(ADC_P15,ADC_SYSclk_DIV_2);
	*/
	// P14��P15ΪADC�ڱ��ã���һ�������ע�͵�
}

// 2.IO�ڳ�ʼ��
void GPIO_init(void)
{
	gpio_mode(P4_7, GPO_PP);
	P47 = 1;
	//ʹ��
	// �����Ͻ�P47��������5Vģ��Ĺ��磬ֻҪ��P13������Ϊ��������������߼����ȶ����5V��ѹ
}

// 3.PWM��ʼ������������������
void PWM_SMB_init(void)
{
	// pwm_init(MOTOR1_P, 17000, 5000);     //ʹ������P6.0  ���PWMƵ��17000HZ   ռ�ձ�Ϊ��5000/10000�������ٷ�֮50%��PWM
	// pwm_init(MOTOR1_N, 17000, 5000);     //ʹ������P6.1  ���PWMƵ��17000HZ   ռ�ձ�Ϊ��5000/10000�������ٷ�֮50%��PWM
	// MOTOR1_My_a
	pwm_init(R_Motor_P, 17000, 2000);
	pwm_init(R_Motor_N, 17000, 2000);
	// ��������Ϊ���1����Ҫ������PWM����
	// pwm_init(MOTOR2_P, 17000, 5000);     //ʹ������P6.4  ���PWMƵ��17000HZ   ռ�ձ�Ϊ��5000/10000�������ٷ�֮50%��PWM
	// pwm_init(MOTOR2_N, 17000, 5000);     //ʹ������P6.5  ���PWMƵ��17000HZ   ռ�ձ�Ϊ��5000/10000�������ٷ�֮50%��PWM
	pwm_init(L_Motor_P, 17000, 2000);
	pwm_init(L_Motor_N, 17000, 2000);

	// ��������Ϊ���2����Ҫ������PWM����
	//pwm_init(BUZZER, 2000, 0); // ʹ������P6.3  ���PWMƵ��2000HZ   ռ�ձ�Ϊ��0/10000�������ٷ�֮0��PWM
	// ����Ϊ����������ʼ�����2000HzƵ�ʡ�ռ�ձ�Ϊ0
	pwm_init(STEERING, 50, 725); // ʹ������P7.4  ���PWMƵ��50HZ   ռ�ձ�Ϊ��750/10000�������ٷ�֮7.5���ߵ�ƽʱ��Ϊ1.5ms����PWM���������
								 // ����Ϊ����������ҪƵ��һ����PWM���������ߵ�ƽʱ���Ӧ��Ƕ�
}

// 4.��ʱ�жϣ��������������У���ʼ��
void PIT_init(void)
{
	pit_timer_ms(TIM_4, 20); // ʹ�ü�ʱ��4��ÿ20ms����һ���ж�
}

// 5.PID��ʼ��
void PID_init(void)
{

	pid_steering.p_steering = 25.5;
	pid_steering.i_steering = 0.05;
	pid_steering.d_steering = 3.2;
	pid_steering.imax = 1;
	pid_steering.imin = -1;
	pid_steering.PID_STEERING_OUT = 735;
	pid_steering.STEERING_OUT_temp = 0.0;

	pid_motor.p_motor = 0.09;		   // ���ڴ�ű���ϵ��p
	pid_motor.i_motor = 0.1;	   // ���ڴ�Ż���ϵ��i
	pid_motor.d_motor = 0.02;		   // ���ڴ��΢��ϵ��d
	pid_motor.PID_MOTOR_L_OUT = 0; // ���ڴ��������������������PWM����ֵ
	pid_motor.PID_MOTOR_R_OUT = 0; // ���ڴ��������������������PWM����ֵ
	pid_motor.MOTOR_L_OUT_temp=0.0;
	pid_motor.MOTOR_R_OUT_temp=0.0;
}

// 6.�����ʼ��
void Steering_init(void)
{
	PWM_Steering_now = 740;
	PWM_Steering_Max = 805; // �����СֵΪʵ�����
	PWM_Steering_Min = 650;
}

// 7.�����ʼ��
void Motor_init(void)
{
	speed_state.Outgar_speed_L_ai = 0; // ������
	speed_state.Outgar_speed_R_ai = 0; // ������

	speed_state.Strai_speed_L_ai = 325; // ֱ����  //ȫ��uint32
	speed_state.Strai_speed_R_ai = 325; // ֱ����

	speed_state.Cur_L_speed_L_ai = 260; // ��ת��
	speed_state.Cur_L_speed_R_ai = 300; // 
	
	speed_state.Cur_R_speed_L_ai = 300; // ��ת��
	speed_state.Cur_R_speed_R_ai = 260; // **����**
	
	speed_state.Ring_speed_L_ai = 150; // Բ���ڲ���
	speed_state.Ring_speed_R_ai = 150; // Բ���ڲ���

	speed_state.Ringin_speed_L_ai = 140; // ��Բ����
	speed_state.Ringin_speed_R_ai = 210; // ��Բ����

	speed_state.Ringout_speed_L_ai = 100; // ��Բ����
	speed_state.Ringout_speed_R_ai = 120; // ��Բ����
//*********************************************
	speed_state.Cross_speed_L_ai = 0; // ʮ����
	speed_state.Cross_speed_R_ai = 0; // ʮ����

	speed_state.Rampin_speed_L_ai = 0; // ���µ���
	speed_state.Rampin_speed_R_ai = 0; // ���µ���

	speed_state.Ramp_speed_L_ai = 0; // �µ���
	speed_state.Ramp_speed_R_ai = 0; // �µ���



	speed_state.Three_speed_L_ai = 0; // ������
	speed_state.Three_speed_R_ai = 0; // ������

	speed_state.Threein_speed_L_ai = 0; // ��������
	speed_state.Threein_speed_R_ai = 0; // ��������

	speed_now.speed_L = 0;	  // ���ֵ�ǰ�ٶ�ֵ
	speed_now.speed_L_ai = 0; // ����Ŀ��ֵ
	speed_now.speed_R = 0;	  // ���ֵ�ǰ�ٶ�ֵ
	speed_now.speed_R_ai = 0; // ����Ŀ��ֵ

	PWM_Motor_Max = 2780;
	PWM_Motor_Min = 0;
	PWM_Motor_L_now = 0;
	PWM_Motor_R_now = 0;
}

// 8.��������ʼ���������ڣ�
void Encoder_init(void)
{
	ctimer_count_init(Encoder_L);
	ctimer_count_init(Encoder_R);
	err_motor.err_L_m = 0;			   // ���ݵ�ǰ��������������������ƫ��ֵ
	err_motor.err_last_L_m = 0;		   // ���ڴ����һ�μ����������ƫ��ֵ
	err_motor.err_past_L_m = 0;		   // ���ڴ������һ�μ����������ƫ��ֵ
	err_motor.err_derivative_L_m = 0;  // ���ڴ�����ֱ���ƫ�����ϴ�ƫ��֮��
	err_motor.err_derivative2_L_m = 0; // ���ڴ�������ϴ�ƫ�������ϴ�ƫ��֮��

	err_motor.err_R_m = 0;			   // ���ݵ�ǰ��������������������ƫ��ֵ
	err_motor.err_last_R_m = 0;		   // ���ڴ����һ�μ����������ƫ��ֵ
	err_motor.err_past_R_m = 0;		   // ���ڴ������һ�μ����������ƫ��ֵ
	err_motor.err_derivative_R_m = 0;  // ���ڴ�����ֱ���ƫ�����ϴ�ƫ��֮��
	err_motor.err_derivative2_R_m = 0; // ���ڴ�������ϴ�ƫ�������ϴ�ƫ��֮��
}
// 9.·���жϱ�־λ��ʼ��
//void FLAG_init(void)
//{
//}

// 10.���ߴ��ڳ�ʼ��
void WIRELESS_init(void)
{
	// seekfree_wireless_init();
}
// 11.���Ա�����ʼ��
void temp_init(void){
	temp = 0.0;
	tempVar = 0.0;
	tempVar1=0.0;
	tempVar2=0.0;
	timer.time0_0=0;
	timer.time0_1=0;
	timer.time1_0=0;
	timer.time1_1=0;
	//ringInFlag = 0;
}
// 12.FLAG��־��ʼ��
void flag_init(void){
	road_flag.Ring_In_Flag =0;
	road_flag.Ring_Out_Flag =0;
	road_flag.Near_Flag=0;
	road_flag.Cross_Flag_Last=0;
	road_flag.Cross_Flag=0;

}

// 13.�����ʼ��
void ALL_init(void)
{

	ADC_all_init();
	GPIO_init();
	PWM_SMB_init();
	PIT_init();
	PID_init();
	Steering_init();
	Motor_init();
	Encoder_init();
	//FLAG_init();
	
	wireless_uart_init();
	//wireless_ch573_init();
	temp_init();
	flag_init();
	//	seekfree_wireless_init();
	// WIRELESS_init();
}