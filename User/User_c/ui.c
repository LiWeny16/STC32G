//���ļ����ڱ�дUIʹ�õ��ĺ�����UI����������OLED��ʹ�õĺ���������������ADCɨ�裬����������
#include "ui.h"


void find_mm_adc(DG_State* dg)
{
	unsigned short ZXM,ZHM,ZSM,YXM,YHM,YSM;//���ڶ�ȡ���м����
	ZXM = adc_once(ZX,RE);
	ZHM = adc_once(ZH,RE);
//	ZSM = adc_once(ZS,RE);
	//YSM = adc_once(YS,RE);
	YHM = adc_once(YH,RE);
	YXM = adc_once(YX,RE);//��ȡ
	
	if(ZXM>=dg->L_zx_max) dg->L_zx_max=ZXM;
	if(ZHM>=dg->L_zh_max) dg->L_zh_max=ZHM;
	if(ZSM>=dg->L_zs_max) dg->L_zs_max=ZSM;
	if(YSM>=dg->L_ys_max) dg->L_ys_max=YSM;
	if(YHM>=dg->L_yh_max) dg->L_yh_max=YHM;
	if(YXM>=dg->L_yx_max) dg->L_yx_max=YXM;//�����ֵ


}