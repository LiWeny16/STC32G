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
		
		case(Curve_Left)://�����
		speed_now-> speed_L_ai = speed_state->Cur_L_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Cur_L_speed_R_ai;
		break;
		
		case(Curve_Right)://�����
		speed_now-> speed_L_ai = speed_state->Cur_R_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Cur_R_speed_R_ai;
		break;
		case(Ring_Left): //����
		speed_now-> speed_L_ai = speed_state->Ring_L_speed_L_ai;//����
		speed_now-> speed_R_ai = speed_state->Ring_L_speed_R_ai;//����
			break;
		
		case(Ring_Right): //����
		speed_now-> speed_L_ai = speed_state->Ring_R_speed_L_ai;//����
		speed_now-> speed_R_ai = speed_state->Ring_R_speed_R_ai;//����
			break;
		
		case(Ring_In): //����Բ��
		speed_now-> speed_L_ai = speed_state->Ringin_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Ringin_speed_R_ai;
			break;
		
		case(Ring_Out): //����Բ��
		speed_now-> speed_L_ai = speed_state->Ringout_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Ringout_speed_R_ai;
			break;
		case(OutGarage):
		speed_now-> speed_L_ai = speed_state->Outgar_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Outgar_speed_R_ai;
			break;
		case(Stop):
		speed_now-> speed_L_ai =0;
		speed_now-> speed_R_ai =0;
			break;
		case(InGarage):
		speed_now-> speed_L_ai =70;
		speed_now-> speed_R_ai =100;
		  break;
		default://���Ĭ��
	  speed_now-> speed_L_ai = speed_state->Strai_speed_L_ai;
		speed_now-> speed_R_ai = speed_state->Strai_speed_R_ai;
			break;
	}
}

void speed_cal(FOOT_COUNTER *foot_counter, SPEED_now* speed_now)//ͨ�������������ݼ����ٶȣ�������д�뵱ǰ�ٶȽṹ��
{
	int Speed_L = 0;
	int Speed_R = 0;
	int dat_L = 0 ;
	int dat_R = 0 ;
	if(DIR_L == 1)//�����ٶ�
		{
			dat_L = ctimer_count_read(Encoder_L);
		//	Speed_L= MFBL*(dat_L/(CONTROL_T*DECO));//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			//Speed_L = dat_L; //������
			//�˲�
		//	Speed_L = Speed_L>1?Speed_L:1;
			Speed_L = dat_L;
		}
		else
		{
			dat_L = ctimer_count_read(Encoder_L) * -1;
		 // Speed_L = (MFBL*(dat_L/(CONTROL_T*DECO))) * -1;//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			//Speed_L = dat_L;
			//Speed_L = Speed_L>1?Speed_L:1;
			Speed_L = dat_L;
		}

		speed_now->speed_L = Speed_L;
		
		
	if(DIR_R == 0)//�����ٶ�
		{
			dat_R = ctimer_count_read(Encoder_R);
		//	Speed_R = MFBL*(dat_R/(CONTROL_T*DECO));//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			//Speed_R = dat_R;
		//	Speed_R = Speed_R>1?Speed_R:1;
			Speed_R = dat_R;
		}
		else
		{
			dat_R = ctimer_count_read(Encoder_R) * -1;
		//	Speed_R = (MFBL*(dat_R/(CONTROL_T*DECO))) * -1;//�ñ��������жϼ��֮�ڵĶ�������ִ�г�����жϼ����һ�׶�Ӧ������������ʾ��ǰ�ٶ�(m/s),�ѳ�PID�ֱ���
			//Speed_R = dat_R;
		//	Speed_R = Speed_R>1?Speed_R:1;
			Speed_R = dat_R;
		}
		//******��********��************
		//������1
		if(foot_counter->speed_counter0_EN==1){
				foot_counter->speed_counter0_0+=Speed_R;	
				if(foot_counter->speed_counter0_0>=10000){
					foot_counter->speed_counter0_1++;
					foot_counter->speed_counter0_0=0;
				}
				if(foot_counter->speed_counter0_1>=10000){
					foot_counter->speed_counter0_2++;
					foot_counter->speed_counter0_1=0;
				}
				if(foot_counter->speed_counter0_2>=10000){
					foot_counter->speed_counter0_2=0;
				}
		}
		else{
		foot_counter->speed_counter0_EN=0;
		foot_counter->speed_counter0_0=0;
		foot_counter->speed_counter0_1=0;
		foot_counter->speed_counter0_2=0;
		}
		//������2
		if(foot_counter->speed_counter1_EN==1){
				foot_counter->speed_counter1_0+=Speed_R;	
				if(foot_counter->speed_counter1_0>=10000){
					foot_counter->speed_counter1_1++;
					foot_counter->speed_counter1_0=0;
				}
				if(foot_counter->speed_counter1_1>=10000){
					foot_counter->speed_counter1_2++;
					foot_counter->speed_counter1_1=0;
				}
				if(foot_counter->speed_counter1_2>=10000){
					foot_counter->speed_counter1_2=0;
				}
		}
		else{
		foot_counter->speed_counter1_EN=0;
		foot_counter->speed_counter1_0=0;
		foot_counter->speed_counter1_1=0;
		foot_counter->speed_counter1_2=0;
		}
		

		//******��********��************
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

