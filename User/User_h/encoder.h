//该文件用于速度状态结构体即电机偏差值结构体的申明以及同名c文件中相应函数的申明

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "common.h"
#include "zf_tim.h"//该文件主要用于读取编码器数据
#include "road.h"
#include "headfile.h"

#define DIR_L P35 //方向
#define DIR_R P36 
#define Encoder_L CTIM0_P34
#define Encoder_R CTIM3_P04
#define CONTROL_T 0.02  //该值需保持为浮点数
//控制周期的倒数（即控制用定时器的中断周期，单位为s），定义以计算速度，如控制周期20ms，则该值为0.02
#define DECO 11587
//小车走1m时编码器的脉冲数
#define MFBL 10000
//定义PID控制的精度数量级，按该值0的个数取，如为10000，则速度误差运算后数量级为千


typedef struct//速度状态结构体（以速度表示）
{
	      //   人为设定的目标速度值
        uint32 Outgar_speed_L_ai;//出库左
        uint32 Outgar_speed_R_ai;//出库右
	////////////我////////////用////////////的//////////////
        uint32 Strai_speed_L_ai;//直道左
        uint32 Strai_speed_R_ai;//直道右
	
	      uint32 Cur_L_speed_L_ai;//左转 左右轮
        uint32 Cur_L_speed_R_ai;//
	
        uint32 Cur_R_speed_L_ai;//右转 左右轮
        uint32 Cur_R_speed_R_ai;//
				
				uint32 Ring_speed_L_ai;//圆环内部左
	      uint32 Ring_speed_R_ai;//圆环内部右
				
        uint32 Ringin_speed_L_ai;//进圆环左
				uint32 Ringin_speed_R_ai;//进圆环右
	
					
        uint32 Ringout_speed_L_ai;//出圆环左
				uint32 Ringout_speed_R_ai;//出圆环右
				
	////////////我////////////用////////////的//////////////
        uint32 Cross_speed_L_ai;//十字左
        uint32 Cross_speed_R_ai;//十字右
	
        uint32 Rampin_speed_L_ai;//上坡道左
        uint32 Rampin_speed_R_ai;//上坡道右
	
        uint32 Ramp_speed_L_ai;//坡道左
	      uint32 Ramp_speed_R_ai;//坡道右
	


				
        uint32 Three_speed_L_ai;//三岔左      
				uint32 Three_speed_R_ai;//三岔右
				
        uint32 Threein_speed_L_ai;//进三岔左
				uint32 Threein_speed_R_ai;//进三岔右
        
} SPEED_state;

typedef struct//增量式PID（电机用）偏差值结构体
{

    int32 err_L_m;//根据当前解码器输出计算出的左轮偏差值
    int32 err_last_L_m;//用于存放上一次计算出的左轮偏差值
    int32 err_past_L_m;//用于存放上上一次计算出的左轮偏差值
	  int32 err_derivative_L_m;//用于存放左轮本次偏差与上次偏差之差
	  int32 err_derivative2_L_m;//用于存放左轮上次偏差与上上次偏差之差

	  int32 err_R_m;//根据当前解码器输出计算出的右轮偏差值
    int32 err_last_R_m;//用于存放上一次计算出的右轮偏差值
    int32 err_past_R_m;//用于存放上上一次计算出的右轮偏差值
		int32 err_derivative_R_m;//用于存放右轮本次偏差与上次偏差之差
	  int32 err_derivative2_R_m;//用于存放右轮上次偏差与上上次偏差之差
	
}Err_Motor;

typedef struct//当前速度结构体（表明当前状态下车的运行状态，值均已速度表示）
{
	uint32 speed_L;//左轮当前速度值
	uint32 speed_L_ai;//左轮目标值
	uint32 speed_R;//右轮当前速度值
	uint32 speed_R_ai;//右轮目标值
}SPEED_now; 


void speedout(Road road,SPEED_now* speed_now,SPEED_state* speed_state);//根据道路情况将速度状态结构体中的一组设定值赋给当前速度结构体中的目标值
void speed_cal(SPEED_now* speed_now);//根据编码器数据计算速度值，并将其赋给当前速度结构体中的当前速度值
void calculate_err_m(SPEED_now* speed_now,Err_Motor* err_Mot);//根据编码器数值计算偏差值，并更新偏差值结构体中last量,结果保存在电机偏差值结构体中
#endif