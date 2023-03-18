// 该文件用于电感状态结构体即舵机偏差值结构体的申明以及同名c文件中相应函数的申明

#ifndef __ADC_H_
#define __ADC_H_

#include "common.h"
#include "math.h"
#include "zf_adc.h"

#define ZH ADC_P00 // L1
#define ZX ADC_P01 // L2

#define YX ADC_P05 // L6
#define YH ADC_P06 // L7

// #define ZX ADC_P00  // L3
// #define ZH ADC_P01  // L1

// #define ZS ADC_P05  // L2 //没用
// #define YS ADC_P06  // L6 //没用

// #define YH ADC_P11  // L7
// #define YX ADC_P10  // L5

#define RE ADC_12BIT // 分辨率
#define count_dg 10 // 平均处理的采样次数
typedef struct      // 电感状态结构体
{

    unsigned short L_zx_real; // 左45度电感
    unsigned short L_zh_real; // 左横电感
    //unsigned short L_zs_real; // 左竖电感
    //unsigned short L_ys_real; // 右竖电感
    unsigned short L_yh_real; // 右横电感
    unsigned short L_yx_real; // 右45度电感
     ///float L_zh_real; // 左横电感
     ///float L_zx_real; // 左45度电感
     ///float L_yh_real; // 右横电感
     ///float L_yx_real; // 右45度电感

    float L_zx_max; // 检测到的最大值
    float L_zh_max;
    float L_yh_max;
    float L_yx_max;
    // unsigned short L_zx_max; // 检测到的最大值
    // unsigned short L_zh_max;
    // unsigned short L_zs_max;
    // unsigned short L_ys_max;
    // unsigned short L_yh_max;
    // unsigned short L_yx_max;

    float L_zx_once; // 一次归一化后
    float L_zh_once;
    float L_zs_once;
    float L_ys_once;
    float L_yh_once;
    float L_yx_once;

    float L_zx_twice; // 两次归一化后
    float L_zh_twice;
    float L_zs_twice;
    float L_ys_twice;
    float L_yh_twice;
    float L_yx_twice;

} DG_State;

typedef struct // 位置式PID（舵机用）偏差值结构体
{
    float Err_x; // 斜电感偏差
    float Err_h; // 横电感偏差
    float Err_s; // 竖电感偏差

    float Err;        // 根据当前电感输出计算出的总偏差值
    float Err_last;   // 用于存放上一次计算出的偏差值
    float Err_x_last; // 用于存放上一次斜电感计算出的偏差值
    float Err_h_last; // 用于存放上一次横电感计算出的偏差值
    float Errsum;     // 用于存放Err累加的和
    float Errdif;     // 用于存放Err的差分
} Err_Steering;

void calculate_s(DG_State *dg, Err_Steering *Err_Ste); // 根据电感状态计算偏差值，并更新偏差值结构体中last量,结果保存在舵机偏差值结构体中

#endif