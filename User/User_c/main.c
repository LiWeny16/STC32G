/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "Allinit.h"

// #include "All_init.h"

/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */
uint16 adc_data[10];
void main()
{
	DisableGlobalIRQ();
	board_init(); // 初始化寄存器,勿删除此句代码。
	ALL_init();
	delay_ms(1000);
	// pwm_init(PWMA_CH1P_P60, 10000, 0);
	// pwm_init(PWMA_CH2P_P62, 10000, 0);
	// pwm_init(PWMA_CH3P_P64, 10000, 0);
	// pwm_init(PWMA_CH4P_P66, 10000, 0);
	EnableGlobalIRQ();
	while (1)
	{
		delay_ms(1000);
		// P52 = !P52;
		// pwm_duty(PWMA_CH1P_P60, 0);
		// pwm_duty(PWMA_CH2P_P62, 1000); // right
		// pwm_duty(PWMA_CH3P_P64, 0);
		// pwm_duty(PWMA_CH4P_P66, 1000);

		//	adc_data[3] = adc_once(ADC_P00, ADC_12BIT); // 采集一次ADC，精度12位
		//	adc_data[4] = adc_once(ADC_P01, ADC_12BIT); // 采集一次ADC，精度12位
		//	adc_data[5] = adc_once(ADC_P05, ADC_12BIT); // 采集一次ADC，精度12位
		// adc_data[6] = adc_once(ADC_P06, ADC_12BIT); // 采集一次ADC，精度12位
		//	printf("adc_data[3] = %d\r\n", adc_data[3]);
		// printf("adc_data[4] = %d\r\n", adc_data[4]);
		// printf("adc_data[5] = %d\r\n", adc_data[5]);
		// printf("adc_data[6] = %d\r\n", adc_data[6]);
		//  Err_x; // 斜电感偏差
		// Err_h; // 横电感偏差
		// Err_s; // 竖电感偏差 没用

		// Err; // 根据当前电感输出计算出的总偏差值
		// Err_last;
		printf("DG_State.L_yx_real = %d\r\n", dg_state.L_yx_real);
		printf("DG_State.L_zx_real = %d\r\n", dg_state.L_zx_real);

		printf("DG_State.L_yh_real = %d\r\n", dg_state.L_yh_real);
		printf("DG_State.L_zh_real = %d\r\n", dg_state.L_zh_real);

		printf("DG_State.L_zx_once = %f\r\n", dg_state.L_zx_once);
		printf("DG_State.L_zh_once = %f\r\n", dg_state.L_zh_once);

		printf("DG_State.L_yx_once = %f\r\n", dg_state.L_yx_once);
		printf("DG_State.L_yh_once = %f\r\n", dg_state.L_yh_once);

		printf("dg_state.L_zx_max = %f\r\n", dg_state.L_zx_max);
		printf("dg_state.L_zh_max = %f\r\n", dg_state.L_zh_max);

		printf("dg_state.L_yh_max = %f\r\n", dg_state.L_yh_max);
		printf("dg_state.L_yx_max = %f\r\n", dg_state.L_yx_max);

		printf("err_steering.Err_x = %f\r\n", err_steering.Err_x);
		printf("err_steering.Err_h = %f\r\n", err_steering.Err_h);

		printf("pid_steering.p_steering = %f\r\n", pid_steering.p_steering);
		printf("pid_steering.i_steering= %f\r\n", pid_steering.i_steering);
		printf("pid_steering.d_steering= %f\r\n", pid_steering.d_steering);
		printf("pid_steering.imax= %f\r\n", pid_steering.imax);
		// printf("err_steering.Err_x = %d\r\n", err_steering.Err_x);
		// printf("err_steering.Err_h = %d\r\n", err_steering.Err_h);

		printf("err_steering.Err = %f\r\n", err_steering.Err);
		printf("err_steering.Err_last = %f\r\n", err_steering.Err_last);
		printf("err_steering.Errsum = %f\r\n",err_steering.Errsum);
		printf("err_steering.Errdif = %f\r\n",err_steering.Errdif);
		
		printf("temp = %f\r\n",temp);
		
		printf("pid_steering.PID_STEERING_OUT=%d \r \n", pid_steering.PID_STEERING_OUT);
		delay_ms(1000);

		// printf("SP_R = %d\r\n", speed_now.speed_R);				// 当前
		// printf("SP_RA = %d\r\n", speed_state.Strai_speed_R_ai); // 目标
		// printf("pwm = %d\r\n\r\n", PWM_Motor_R_now);

		//	printf("P = %f\r\n", pid_motor.p_motor);
		//	printf("I = %f\r\n", pid_motor.i_motor);
		// printf("D = %f\r\n\r\n", pid_motor.d_motor);

		// printf("PWM_L = %d \r\n\r\n", PWM_Motor_L_now);
		//	printf("PWM_R = %d \r\n\r\n", PWM_Motor_R_now);
		printf("\r\n\r\n");
	}
}
