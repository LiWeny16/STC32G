// 该文件用于舵机PID结构体和电机PID结构体的申明以及同名c文件中函数的申明

#ifndef __PID_H_
#define __PID_H_

#include "common.h"
#include "adc.h"
#include "encoder.h"
//
#include "TempVar.h"

//#include "headfile.h"

typedef struct // 位置式PID（舵机用）结构体
{

    float p_steering;       // 用于存放比例系数p
    float i_steering;       // 用于存放积分系数i
    float d_steering;       // 用于存放微分系数d
    float imax;             // 积分限幅上限
    float imin;             // 积分限幅下限
	  float STEERING_OUT_temp;
    uint32 PID_STEERING_OUT; // 用于存放最终输出给舵机的PWM值
} PID_Steering;

typedef struct // 增量式PID（电机用）结构体
{

    float p_motor;         // 用于存放比例系数p
    float i_motor;         // 用于存放积分系数i
    float d_motor;         // 用于存放微分系数d
    int32 PID_MOTOR_L_OUT; // 用于存放最终输出给电机的左轮PWM增量值
    int32 PID_MOTOR_R_OUT; // 用于存放最终输出给电机的右轮PWM增量值
} PID_Motor;

uint32 constrain_uint32(uint32 amt, uint32 low, uint32 high);
float constrain_float(float amt, float low, float high);                             // 位置式PID积分项限幅及PWM输出限幅用
void Pid_Steering_Calculate(Err_Steering *err_steering, PID_Steering *pid_steering); // 由舵机偏差（由电感反应）结构体计算舵机PID输出值
void Pid_Motor_Calculate(Err_Motor *err_motor, PID_Motor *pid_motor);                // 由电机偏差（由编码器反应）结构体计算电机PID输出增量
#endif