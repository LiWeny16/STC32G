// 该文件用于将各部分初始化，并将其整合为一个函数Allinit（）

#include "Allinit.h"
// 1.电感ADC模块初始化函数（初始化各IO口为ADC采样口，并定义采样频率为SYSclk的一半）
void ADC_all_init(void)
{
	adc_init(ADC_P00, ADC_SYSclk_DIV_2);
	adc_init(ADC_P01, ADC_SYSclk_DIV_2);
	adc_init(ADC_P05, ADC_SYSclk_DIV_2);
	adc_init(ADC_P06, ADC_SYSclk_DIV_2);
	// adc_init(ADC_P11, ADC_SYSclk_DIV_2);
	// adc_init(ADC_P10, ADC_SYSclk_DIV_2);

	err_steering.Err_x = 0; // 舵机偏差结构体（实参）
	err_steering.Err_h = 0;
	err_steering.Err_s = 0;
	err_steering.Err = 0;
	err_steering.Err_last = 0;
	err_steering.Err_x_last = 0;
	err_steering.Err_h_last = 0;
	err_steering.Errsum = 0;
	err_steering.Errdif = 0;

	dg_state.L_zx_real = 0; // 电感状态结构体（实参）
	dg_state.L_zh_real = 0;
//	dg_state.L_zs_real = 0;
	//dg_state.L_ys_real = 0;
	dg_state.L_yh_real = 0;
	dg_state.L_yx_real = 0;

	// dg_state.L_zx_max = 33; // 检测到的最大值
	// dg_state.L_zh_max = 2183;
	// dg_state.L_zs_max = 0;
	// dg_state.L_ys_max = 0;
	// dg_state.L_yh_max = 1831;
	// dg_state.L_yx_max = 1700;
	dg_state.L_zx_max = 4096.0; // 检测到的最大值
	dg_state.L_zh_max = 4096.0;
//	dg_state.L_zs_max = 0;
//	dg_state.L_ys_max = 0;
	dg_state.L_yh_max = 4096.0;
	dg_state.L_yx_max = 4096.0;

	dg_state.L_zx_once = 0; // 一次归一化后
	dg_state.L_zh_once = 0;
	dg_state.L_zs_once = 0;
	dg_state.L_ys_once = 0;
	dg_state.L_yh_once = 0;
	dg_state.L_yx_once = 0;

	dg_state.L_zx_twice = 0; // 两次归一化后
	dg_state.L_zh_twice = 0;
	dg_state.L_zs_twice = 0;
	dg_state.L_ys_twice = 0;
	dg_state.L_yh_twice = 0;
	dg_state.L_yx_twice = 0;

	/*
	adc_init(ADC_P14,ADC_SYSclk_DIV_2);
	adc_init(ADC_P15,ADC_SYSclk_DIV_2);
	*/
	// P14、P15为ADC口备用，第一版程序先注释掉
}

// 2.IO口初始化
void GPIO_init(void)
{
	gpio_mode(P1_3, GPO_PP);
	P13 = 1;
	// 板子上将P13用作其他5V模块的供电，只要将P13口设置为推挽输出，并赋高即可稳定输出5V电压
}

// 3.PWM初始化（舵机电机蜂鸣器）
void PWM_SMB_init(void)
{
	// pwm_init(MOTOR1_P, 17000, 5000);     //使用引脚P6.0  输出PWM频率17000HZ   占空比为（5000/10000），即百分之50%的PWM
	// pwm_init(MOTOR1_N, 17000, 5000);     //使用引脚P6.1  输出PWM频率17000HZ   占空比为（5000/10000），即百分之50%的PWM
	// MOTOR1_My_a
	pwm_init(PWMA_CH1P_P60, 17000, 1000);
	pwm_init(PWMA_CH2P_P62, 17000, 0);
	// 以上两组为电机1，需要互反的PWM驱动
	// pwm_init(MOTOR2_P, 17000, 5000);     //使用引脚P6.4  输出PWM频率17000HZ   占空比为（5000/10000），即百分之50%的PWM
	// pwm_init(MOTOR2_N, 17000, 5000);     //使用引脚P6.5  输出PWM频率17000HZ   占空比为（5000/10000），即百分之50%的PWM
	pwm_init(PWMA_CH3P_P64, 17000, 0);
	pwm_init(PWMA_CH4P_P66, 17000, 0);

	// 以上两组为电机2，需要互反的PWM驱动
	pwm_init(BUZZER, 2000, 0); // 使用引脚P6.3  输出PWM频率2000HZ   占空比为（0/10000），即百分之0的PWM
	// 以上为蜂鸣器，初始化输出2000Hz频率。占空比为0
	pwm_init(STEERING, 50, 725); // 使用引脚P7.4  输出PWM频率50HZ   占空比为（750/10000），即百分之7.5（高电平时间为1.5ms）的PWM，舵机归中
								 // 以上为舵机，舵机需要频率一定的PWM驱动，靠高电平时间对应其角度
}

// 4.定时中断（用于主程序运行）初始化
void PIT_init(void)
{
	pit_timer_ms(TIM_4, 20); // 使用计时器4，每20ms进入一次中断
}

// 5.PID初始化
void PID_init(void)
{

	pid_steering.p_steering = 22.5;
	pid_steering.i_steering = 0.0;
	pid_steering.d_steering = 5.5;
	pid_steering.imax = 1;
	pid_steering.imin = -1;
	pid_steering.PID_STEERING_OUT = 725;
	pid_steering.STEERING_OUT_temp = 0.0;

	pid_motor.p_motor = 0.05;		   // 用于存放比例系数p
	pid_motor.i_motor = 0.002;	   // 用于存放积分系数i
	pid_motor.d_motor = 0.01;		   // 用于存放微分系数d
	pid_motor.PID_MOTOR_L_OUT = 0; // 用于存放最终输出给电机的左轮PWM增量值
	pid_motor.PID_MOTOR_R_OUT = 0; // 用于存放最终输出给电机的右轮PWM增量值
	pid_motor.MOTOR_L_OUT_temp=0.0;
	pid_motor.MOTOR_R_OUT_temp=0.0;
}

// 6.舵机初始化
void Steering_init(void)
{
	PWM_Steering_now = 730;
	PWM_Steering_Max = 805; // 最大最小值为实测参数
	PWM_Steering_Min = 650;
}

// 7.电机初始化
void Motor_init(void)
{
	speed_state.Outgar_speed_L_ai = 0; // 出库左
	speed_state.Outgar_speed_R_ai = 0; // 出库右

	speed_state.Strai_speed_L_ai = 12000; // 直道左  //全是uint32
	speed_state.Strai_speed_R_ai = 12000; // 直道右

	speed_state.Cur_L_speed_L_ai = 9000; // 左转！
	speed_state.Cur_L_speed_R_ai = 11000; // 
	
	speed_state.Cur_R_speed_L_ai = 11000; // 右转！
	speed_state.Cur_R_speed_R_ai = 9000; // 

	speed_state.Cross_speed_L_ai = 0; // 十字左
	speed_state.Cross_speed_R_ai = 0; // 十字右

	speed_state.Rampin_speed_L_ai = 0; // 上坡道左
	speed_state.Rampin_speed_R_ai = 0; // 上坡道右

	speed_state.Ramp_speed_L_ai = 0; // 坡道左
	speed_state.Ramp_speed_R_ai = 0; // 坡道右

	speed_state.Ring_speed_L_ai = 0; // 圆环内部左
	speed_state.Ring_speed_R_ai = 0; // 圆环内部右

	speed_state.Ringin_speed_L_ai = 0; // 进圆环左
	speed_state.Ringin_speed_R_ai = 0; // 进圆环右

	speed_state.Ringout_speed_L_ai = 0; // 出圆环左
	speed_state.Ringout_speed_R_ai = 0; // 出圆环右

	speed_state.Three_speed_L_ai = 0; // 三岔左
	speed_state.Three_speed_R_ai = 0; // 三岔右

	speed_state.Threein_speed_L_ai = 0; // 进三岔左
	speed_state.Threein_speed_R_ai = 0; // 进三岔右

	speed_now.speed_L = 0;	  // 左轮当前速度值
	speed_now.speed_L_ai = 0; // 左轮目标值
	speed_now.speed_R = 0;	  // 右轮当前速度值
	speed_now.speed_R_ai = 0; // 右轮目标值

	PWM_Motor_Max = 3000;
	PWM_Motor_Min = 0;
	PWM_Motor_L_now = 2000;
	PWM_Motor_R_now = 2000;
}

// 8.编码器初始化（带串口）
void Encoder_init(void)
{
	ctimer_count_init(Encoder_L);
	ctimer_count_init(Encoder_R);
	err_motor.err_L_m = 0;			   // 根据当前解码器输出计算出的左轮偏差值
	err_motor.err_last_L_m = 0;		   // 用于存放上一次计算出的左轮偏差值
	err_motor.err_past_L_m = 0;		   // 用于存放上上一次计算出的左轮偏差值
	err_motor.err_derivative_L_m = 0;  // 用于存放左轮本次偏差与上次偏差之差
	err_motor.err_derivative2_L_m = 0; // 用于存放左轮上次偏差与上上次偏差之差

	err_motor.err_R_m = 0;			   // 根据当前解码器输出计算出的右轮偏差值
	err_motor.err_last_R_m = 0;		   // 用于存放上一次计算出的右轮偏差值
	err_motor.err_past_R_m = 0;		   // 用于存放上上一次计算出的右轮偏差值
	err_motor.err_derivative_R_m = 0;  // 用于存放右轮本次偏差与上次偏差之差
	err_motor.err_derivative2_R_m = 0; // 用于存放右轮上次偏差与上上次偏差之差
}
// 9.路况判断标志位初始化
void FLAG_init(void)
{
}

// 10.无线串口初始化
void WIRELESS_init(void)
{
	// seekfree_wireless_init();
}
// 11.调试变量初始化
void temp_init(void){
	temp = 0.0;
	tempVar = 0.0;
	ringInFlag = 0;
}

// 11.总体初始化
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
	FLAG_init();
	
	wireless_uart_init();
	//wireless_ch573_init();
	temp_init();
	//	seekfree_wireless_init();
	// WIRELESS_init();
}