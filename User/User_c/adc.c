// 该文件用于将使用ADC采集到的电感输出值变化为舵机偏差值，并将相应数据录入舵机偏差结构体中

#include "adc.h"

void calculate_s(DG_State *dg, Err_Steering *Err_Ste) // 根据电感状态计算偏差值，并更新偏差值结构体中last量，结果保存在舵机偏差值结构体中
{
  dg->L_zx_real = (ADC_ReadAverage(ZX, count_dg, RE));
  dg->L_zh_real = (ADC_ReadAverage(ZH, count_dg, RE));

//  dg->L_zs_real = (ADC_ReadAverage(ZS, count_dg, RE));//没用
//  dg->L_ys_real = (ADC_ReadAverage(YS, count_dg, RE));//没用
  
  dg->L_yh_real = (ADC_ReadAverage(YH, count_dg, RE));
  dg->L_yx_real = (ADC_ReadAverage(YX, count_dg, RE));
  // 求取10次采样平均值作为真实值

  dg->L_zx_real = dg->L_zx_real >= 1 ? dg->L_zx_real : 1;
  dg->L_zh_real = dg->L_zh_real >= 1 ? dg->L_zh_real : 1;
  dg->L_zs_real = dg->L_zs_real >= 1 ? dg->L_zs_real : 1;
  dg->L_ys_real = dg->L_ys_real >= 1 ? dg->L_ys_real : 1;
  dg->L_yh_real = dg->L_yh_real >= 1 ? dg->L_yh_real : 1;
  dg->L_yx_real = dg->L_yx_real >= 1 ? dg->L_yx_real : 1;
  // 使采样最小值为1

  dg->L_zx_once = (sqrt(((float)dg->L_zx_real) / (dg->L_zx_max)));
  dg->L_zh_once = (sqrt(((float)dg->L_zh_real) / (dg->L_zh_max)));
  dg->L_zs_once = (sqrt(((float)dg->L_zs_real) / (dg->L_zs_max)));//没用
  dg->L_ys_once = (sqrt(((float)dg->L_ys_real) / (dg->L_ys_max)));//没用
  dg->L_yh_once = (sqrt(((float)dg->L_yh_real) / (dg->L_yh_max)));
  dg->L_yx_once = (sqrt(((float)dg->L_yx_real) / (dg->L_yx_max)));
  // 第一次归一化，使得电感距离电感最大值处的吻合度用百分比表示，并开方使得偏差和小车偏离赛道的距离成单调关系（即消除极点），使得丢线更好判断

  dg->L_zx_twice = (dg->L_zx_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  dg->L_zh_twice = (dg->L_zh_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  dg->L_yh_twice = (dg->L_yh_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  dg->L_yx_twice = (dg->L_yx_once) / (dg->L_zx_once * dg->L_zx_once + dg->L_yx_once * dg->L_yx_once + dg->L_zh_once * dg->L_zh_once + dg->L_yh_once * dg->L_yh_once);
  // 第二次归一化，求取横电感和斜电感偏差值占总的百分比

  Err_Ste->Err_x = dg->L_zx_twice - dg->L_yx_twice;
  Err_Ste->Err_h = dg->L_zh_twice - dg->L_yh_twice;
  Err_Ste->Err = (-0.6151 * (Err_Ste->Err_x)) + (3.3868 * (Err_Ste->Err_h));
  // 以上系数使用matlab拟合，使得偏差和小车偏离赛道的距离成线性关系，至此，偏差和小车偏离赛道的距离成单调线性关系
  Err_Ste->Errsum += Err_Ste->Err;
  Err_Ste->Errdif = Err_Ste->Err - Err_Ste->Err_last;
  // 位置式PID积分和差分运算
  Err_Ste->Err_x_last = Err_Ste->Err_x;
  Err_Ste->Err_h_last = Err_Ste->Err_h;
  Err_Ste->Err_last = Err_Ste->Err;
  // 偏差值更新
}
