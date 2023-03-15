// ���ļ����ڽ�ʹ��ADC�ɼ����ĵ�����ֵ�仯Ϊ���ƫ��ֵ��������Ӧ����¼����ƫ��ṹ����

#include "adc.h"

void calculate_s(DG_State *dg, Err_Steering *Err_Ste) // ���ݵ��״̬����ƫ��ֵ��������ƫ��ֵ�ṹ����last������������ڶ��ƫ��ֵ�ṹ����
{
  dg->L_zx_real = (ADC_ReadAverage(ZX, count_dg, RE));
  dg->L_zh_real = (ADC_ReadAverage(ZH, count_dg, RE));

//  dg->L_zs_real = (ADC_ReadAverage(ZS, count_dg, RE));//û��
//  dg->L_ys_real = (ADC_ReadAverage(YS, count_dg, RE));//û��
  
  dg->L_yh_real = (ADC_ReadAverage(YH, count_dg, RE));
  dg->L_yx_real = (ADC_ReadAverage(YX, count_dg, RE));
  // ��ȡ10�β���ƽ��ֵ��Ϊ��ʵֵ

  dg->L_zx_real = dg->L_zx_real >= 1 ? dg->L_zx_real : 1;
  dg->L_zh_real = dg->L_zh_real >= 1 ? dg->L_zh_real : 1;
  dg->L_zs_real = dg->L_zs_real >= 1 ? dg->L_zs_real : 1;
  dg->L_ys_real = dg->L_ys_real >= 1 ? dg->L_ys_real : 1;
  dg->L_yh_real = dg->L_yh_real >= 1 ? dg->L_yh_real : 1;
  dg->L_yx_real = dg->L_yx_real >= 1 ? dg->L_yx_real : 1;
  // ʹ������СֵΪ1

  dg->L_zx_once = (sqrt(((float)dg->L_zx_real) / (dg->L_zx_max)));
  dg->L_zh_once = (sqrt(((float)dg->L_zh_real) / (dg->L_zh_max)));
  dg->L_zs_once = (sqrt(((float)dg->L_zs_real) / (dg->L_zs_max)));//û��
  dg->L_ys_once = (sqrt(((float)dg->L_ys_real) / (dg->L_ys_max)));//û��
  dg->L_yh_once = (sqrt(((float)dg->L_yh_real) / (dg->L_yh_max)));
  dg->L_yx_once = (sqrt(((float)dg->L_yx_real) / (dg->L_yx_max)));
  // ��һ�ι�һ����ʹ�õ�о��������ֵ�����Ǻ϶��ðٷֱȱ�ʾ��������ʹ��ƫ���С��ƫ�������ľ���ɵ�����ϵ�����������㣩��ʹ�ö��߸����ж�

  dg->L_zx_twice = (dg->L_zx_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  dg->L_zh_twice = (dg->L_zh_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  dg->L_yh_twice = (dg->L_yh_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  dg->L_yx_twice = (dg->L_yx_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  // �ڶ��ι�һ������ȡ���к�б���ƫ��ֵռ�ܵİٷֱ�

  Err_Ste->Err_x = dg->L_zx_twice - dg->L_yx_twice;
  Err_Ste->Err_h = dg->L_zh_twice - dg->L_yh_twice;
  Err_Ste->Err = (-0.6151 * (Err_Ste->Err_x)) + (3.3868 * (Err_Ste->Err_h));
  // ����ϵ��ʹ��matlab��ϣ�ʹ��ƫ���С��ƫ�������ľ�������Թ�ϵ�����ˣ�ƫ���С��ƫ�������ľ���ɵ������Թ�ϵ
  Err_Ste->Errsum += Err_Ste->Err;
  Err_Ste->Errdif = Err_Ste->Err - Err_Ste->Err_last;
  // λ��ʽPID���ֺͲ������
  Err_Ste->Err_x_last = Err_Ste->Err_x;
  Err_Ste->Err_h_last = Err_Ste->Err_h;
  Err_Ste->Err_last = Err_Ste->Err;
  // ƫ��ֵ����
}
