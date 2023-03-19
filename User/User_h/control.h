// 该文件用于对初始化函数进行申明

#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "common.h"
#include "zf_pwm.h"   //使用其中pwm更改占空比函数
#include "Allinit.h"  //使用其中电机通道声明
#include "Variable.h" //使用其中电机舵机相关全局变量
#include "Pid.h"      //使用其中的限幅函数
#include "adc.h"      //使用其中相关计算函数
#include "encoder.h"  //使用其中相关计算函数
#include "road.h"     //使用道路判断结构体和道路判断函数

void STEERING_Control(Road road,PID_Steering *pid_steering); // 舵机驱动输出（从PID到实际输出）
void MOTOR_Control(Road road,PID_Motor *pid_motor);          // 电机驱动输出（从PID到实际输出）
void Control_All();                                // 总控制封装，包括：电感->偏差->舵机PID->实际驱动,编码器->速度->偏差->电机PID->实际驱动
#endif