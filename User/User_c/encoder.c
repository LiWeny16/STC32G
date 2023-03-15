//���ļ����ڽ�ʹ�ñ������ɼ���������ת��Ϊ�ٶȣ����������ٶȽ��бȽϣ�������ƫ��ֵ��������Ӧ����¼����ƫ��ṹ����

#include "encoder.h"

void speedout(Road road,SPEED_now* speed_now,SPEED_state* speed_state)//����·��ѡ��Ŀ���ٶȲ����丳����ǰ�ٶȽṹ��
{
	switch(road)
	{
		case(Straight)://ֱ��
		speed_now-> speed_L_ai = speed_state->Strai_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Strai_speed_R_ai;
		break;
		
		case(Curve)://���
		speed_now-> speed_L_ai = speed_state->Cur_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Cur_speed_R_ai;
		break;
		
		default://���
	  speed_now-> speed_L_ai = speed_state->Strai_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Strai_speed_R_ai;
		break;
	}
}

void speed_cal(SPEED_now* speed_now)//ͨ�������������ݼ����ٶȣ�������д�뵱ǰ�ٶȽṹ��
{
	uint32 Speed_L = 0;
	uint32 Speed_R = 0;
	uint32 dat_L = 0 ;
	uint32 dat_R = 0 ;
	if(DIR_L == 1)//�����ٶ�
		{
			dat_L = ctimer_count_read(Encoder_L);
			Speed_L = MFBL*(dat_L/(CONTROL_T*DECO));//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			//Speed_L = dat_L;
		}
		else
		{
			dat_L = ctimer_count_read(Encoder_L) * -1;
		//	Speed_L = (MFBL*(dat_L/(CONTROL_T*DECO))) * -1;//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			Speed_L = dat_L;
		}

		speed_now->speed_L = Speed_L;
		
		
	if(DIR_R == 0)//�����ٶ�
		{
			dat_R = ctimer_count_read(Encoder_R);
		//	Speed_R = MFBL*(dat_R/(CONTROL_T*DECO));//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			Speed_R = dat_R;
		}
		else
		{
			dat_R = ctimer_count_read(Encoder_R) * -1;
			Speed_R = (MFBL*(dat_R/(CONTROL_T*DECO))) * -1;//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			//Speed_R = dat_R;
		}

		speed_now->speed_R = Speed_R;
		ctimer_count_clean(Encoder_R);
		ctimer_count_clean(Encoder_L);
}


void calculate_err_m(SPEED_now* speed_now,Err_Motor* err_Mot)//ͨ�����ֺ������ٶȼ������������ʽPID���ֵ������д�������ṹ�壬��������Ӧ���������ֵ
{
	err_Mot->err_L_m = speed_now->speed_L_ai - speed_now->speed_L;
	err_Mot->err_R_m = speed_now->speed_R_ai - speed_now->speed_R;
	
	err_Mot->err_derivative_L_m = err_Mot->err_L_m - err_Mot->err_last_L_m;//����������ϴ����֮��
	err_Mot->err_derivative_R_m = err_Mot->err_R_m - err_Mot->err_last_R_m;
	
	err_Mot->err_derivative2_L_m = err_Mot->err_last_L_m - err_Mot->err_past_L_m;//�ϴ��������ϴ����֮��
	err_Mot->err_derivative2_R_m = err_Mot->err_last_R_m - err_Mot->err_past_R_m;
	
	err_Mot->err_past_L_m = err_Mot->err_last_L_m;
	err_Mot->err_last_L_m = err_Mot->err_L_m;//�������ݸ���
	
	err_Mot->err_past_R_m = err_Mot->err_last_R_m;
	err_Mot->err_last_R_m = err_Mot->err_R_m;//�������ݸ���
	
}

