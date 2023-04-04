//该文件主要用于将需要的变量申明为全局变量（如结构体等）
//（注：结构体定义、结构体申明为全局变量、结构体使用三个部分中结构体申明为全局变量、结构体使用两个部分必须包含结构体定义部分的头文件，如该文件中Err_Steering在adc.h中定义，在Variable.h中申明为全局变量，在Pid.h中使用，则Variable.h和Pid.h需包含adc.h）

#ifndef __VARIABLE_H_
#define __VARIABLE_H_

#include "common.h"
#include "adc.h"
#include "encoder.h"
#include "Pid.h"
#include "TempVar.h"

/**************基础变量******/

extern volatile Err_Steering err_steering;//舵机偏差结构体（实参）
extern volatile DG_State dg_state;//电感状态结构体（实参）
extern volatile PID_Steering pid_steering;//舵机PID结构体（实参）
extern volatile uint16  PWM_Steering_now;//当前舵机占空比
extern volatile uint32  PWM_Steering_Max;//舵机最大占空比
extern volatile uint32  PWM_Steering_Min;//舵机最小占空比



extern volatile SPEED_state speed_state;//电机速度状态结构体，用于存放不同路况时的目标速度（实参）
extern volatile SPEED_now speed_now;//电机当前速度结构体，用于存放左右轮目标速度和当前速度（实参）
extern volatile Err_Motor err_motor;//电机偏差结构体（实参）
extern volatile PID_Motor pid_motor;//电机PID结构体（实参）
extern volatile uint16  PWM_Motor_Max;//电机最大占空比
extern volatile uint16  PWM_Motor_Min;//电机最小占空比
extern volatile uint16  PWM_Motor_L_now;//当前左电机占空比
extern volatile uint16  PWM_Motor_R_now;//当前右电机占空比


extern volatile TIMER timer;
extern volatile float temp;
extern volatile Road road;//道路判断结构体
extern volatile FLAG road_flag;
#endif