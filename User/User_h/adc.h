// ���ļ����ڵ��״̬�ṹ�弴���ƫ��ֵ�ṹ��������Լ�ͬ��c�ļ�����Ӧ����������

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

// #define ZS ADC_P05  // L2 //û��
// #define YS ADC_P06  // L6 //û��

// #define YH ADC_P11  // L7
// #define YX ADC_P10  // L5

#define RE ADC_12BIT // �ֱ���
#define count_dg 10 // ƽ������Ĳ�������
typedef struct      // ���״̬�ṹ��
{

    unsigned short L_zx_real; // ��45�ȵ��
    unsigned short L_zh_real; // �����
    //unsigned short L_zs_real; // �������
    //unsigned short L_ys_real; // �������
    unsigned short L_yh_real; // �Һ���
    unsigned short L_yx_real; // ��45�ȵ��
     ///float L_zh_real; // �����
     ///float L_zx_real; // ��45�ȵ��
     ///float L_yh_real; // �Һ���
     ///float L_yx_real; // ��45�ȵ��

    float L_zx_max; // ��⵽�����ֵ
    float L_zh_max;
    float L_yh_max;
    float L_yx_max;
    // unsigned short L_zx_max; // ��⵽�����ֵ
    // unsigned short L_zh_max;
    // unsigned short L_zs_max;
    // unsigned short L_ys_max;
    // unsigned short L_yh_max;
    // unsigned short L_yx_max;

    float L_zx_once; // һ�ι�һ����
    float L_zh_once;
    float L_zs_once;
    float L_ys_once;
    float L_yh_once;
    float L_yx_once;

    float L_zx_twice; // ���ι�һ����
    float L_zh_twice;
    float L_zs_twice;
    float L_ys_twice;
    float L_yh_twice;
    float L_yx_twice;

} DG_State;

typedef struct // λ��ʽPID������ã�ƫ��ֵ�ṹ��
{
    float Err_x; // б���ƫ��
    float Err_h; // ����ƫ��
    float Err_s; // �����ƫ��

    float Err;        // ���ݵ�ǰ���������������ƫ��ֵ
    float Err_last;   // ���ڴ����һ�μ������ƫ��ֵ
    float Err_x_last; // ���ڴ����һ��б��м������ƫ��ֵ
    float Err_h_last; // ���ڴ����һ�κ��м������ƫ��ֵ
    float Errsum;     // ���ڴ��Err�ۼӵĺ�
    float Errdif;     // ���ڴ��Err�Ĳ��
} Err_Steering;

void calculate_s(DG_State *dg, Err_Steering *Err_Ste); // ���ݵ��״̬����ƫ��ֵ��������ƫ��ֵ�ṹ����last��,��������ڶ��ƫ��ֵ�ṹ����

#endif