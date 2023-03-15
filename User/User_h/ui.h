//该文件用于对初始化函数进行申明

#ifndef __UI_H_
#define __UI_H_
#include "common.h"
#include "adc.h"//使用其中的宏定义

#define SCAN P42
#define CONF P22
#define UP   P24
#define RIGH P27
#define LEFT P26
#define DOWN P76

void find_mm_adc(DG_State* dg);//用于找到电感的最大值和最小值，并将其写入电感结构体中

#endif