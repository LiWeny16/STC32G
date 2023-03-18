//该文件用于对初始化函数进行申明

#ifndef __ALLINIT_H_
#define __ALLINIT_H_

//电机通道声明
#define MOTOR1_P PWMA_CH1P_P60
#define MOTOR1_N PWMA_CH1N_P61
#define MOTOR2_P PWMA_CH3P_P64
#define MOTOR2_N PWMA_CH3N_P65

#define MOTOR1_My_a PWMA_CH1P_P60
#define MOTOR1_My_b PWMA_CH2P_P62
#define MOTOR2_My_a PWMA_CH3P_P64
#define MOTOR2_My_b PWMA_CH4P_P66
//舵机通道声明
#define STEERING PWMB_CH1_P74

//蜂鸣器声明
#define BUZZER  PWMA_CH2N_P63

//编码器串口声明
#define Encoder_L CTIM0_P34 //左轮
#define Encoder_R CTIM3_P04 //右轮

#include "common.h"
#include "zf_adc.h"
#include "zf_tim.h"
#include "zf_gpio.h"
#include "zf_pwm.h"
#include "SEEKFREE_WIRELESS.h"
#include "adc.h"
#include "Pid.h"
#include "road.h"
#include "encoder.h"
#include "Variable.h"
#include "TempVar.h"

void ADC_all_init(void);//电感ADC模块初始化
void GPIO_init(void);//IO口初始化
void PWM_SMB_init(void);//电机、舵机和蜂鸣器的PWM初始化
void PIT_init(void);//定时中断（用于电感检测）初始化
void PID_init(void);//PID初始化
void Steering_init(void);//舵机初始化，归中，设置上下限
void Motor_init(void);//电机初始化
void FLAG_init(void);//路况判断标志位初始化
void WIRELESS_init(void);//无线串口初始化
void ALL_init(void);//总体初始化

void temp_init(void);

#endif