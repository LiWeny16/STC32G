/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "Allinit.h"

// #include "All_init.h"

/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */
uint16 adc_data[10];
void main()
{
	DisableGlobalIRQ();
	board_init(); // ��ʼ���Ĵ���,��ɾ���˾���롣
	ALL_init();
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

		adc_data[3] = adc_once(ADC_P00, ADC_12BIT); // �ɼ�һ��ADC������12λ
		adc_data[4] = adc_once(ADC_P01, ADC_12BIT); // �ɼ�һ��ADC������12λ
		adc_data[5] = adc_once(ADC_P05, ADC_12BIT); // �ɼ�һ��ADC������12λ
		adc_data[6] = adc_once(ADC_P06, ADC_12BIT); // �ɼ�һ��ADC������12λ
		printf("adc_data[3] = %d\r\n", adc_data[3]);
		printf("adc_data[4] = %d\r\n", adc_data[4]);
		printf("adc_data[5] = %d\r\n", adc_data[5]);
		printf("adc_data[6] = %d\r\n", adc_data[6]);
		
		printf("DG_State.L_yh_real = %d\r\n",dg_state.L_yh_real);

		delay_ms(1000);

		printf("SP_R = %d\r\n", speed_now.speed_R);				// ��ǰ
		printf("SP_RA = %d\r\n", speed_state.Strai_speed_R_ai); // Ŀ��
		printf("pwm = %d\r\n\r\n", PWM_Motor_R_now);

	//	printf("P = %f\r\n", pid_motor.p_motor);
	//	printf("I = %f\r\n", pid_motor.i_motor);
		//printf("D = %f\r\n\r\n", pid_motor.d_motor);

		//printf("PWM_L = %d \r\n\r\n", PWM_Motor_L_now);
	//	printf("PWM_R = %d \r\n\r\n", PWM_Motor_R_now);
		printf("\r\n\r\n");
	}
}
