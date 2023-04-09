//该文件用于对各个需要用到的变量进行赋值


#include "Variable.h"

volatile Err_Steering err_steering;//舵机偏差结构体（实参）
volatile DG_State dg_state;//电感状态结构体（实参）
volatile PID_Steering pid_steering;//舵机PID结构体（实参）
volatile uint16  PWM_Steering_now;//当前舵机占空比
volatile uint32  PWM_Steering_Max;//舵机最大占空比
volatile uint32  PWM_Steering_Min;//舵机最小占空比



volatile SPEED_state speed_state;//电机速度状态结构体，用于存放不同路况时的目标速度（实参）
volatile SPEED_now speed_now;//电机当前速度结构体，用于存放左右轮目标速度和当前速度（实参）
volatile Err_Motor err_motor;//电机偏差结构体（实参）
volatile PID_Motor pid_motor;//电机PID结构体（实参）
volatile uint16  PWM_Motor_Max;//电机最大占空比
volatile uint16  PWM_Motor_Min;//电机最小占空比
volatile uint16  PWM_Motor_L_now;//当前左电机占空比
volatile uint16  PWM_Motor_R_now;//当前右电机占空比

volatile TIMER timer;
volatile FOOT_COUNTER foot_counter;
volatile float temp;
volatile FLAG road_flag;
volatile Road road;//道路判断结构体