#include "pwm.h"

/*************************************************************
�������ܣ�PWM��ֵ
��ڲ�����motor_left:����PWMֵ��motor_right:�ҵ��PWMֵ
����  ֵ����
*************************************************************/
//in1����pwm��in2����0����ת
//in1����0��in2����pwm����ת
//����
void Set_PWM_left(int in1,int in2)
{
	TIM3->CCR3 = in1; //����in1,PB0
	TIM3->CCR4 = in2; //����in2,PB1	
}

//����
void Set_PWM_right(int in1,int in2)
{
	TIM5->CCR1 = in1; //�ҵ��in1,PA0
	TIM5->CCR2 = in2; //�ҵ��in2,PA1
}

	 //in1����pwm��in2����0����ת
	 //in1����0��in2����pwm����ת
	 //����
	 //TIM5->CCR1 = 0; //�ҵ��in1,PA0
	 //TIM5->CCR2 = 4200; //�ҵ��in2,PA1
	 //TIM3->CCR3 = 4200; //����in1,PB0
	 //TIM3->CCR4 = 0; //����in2,PB1
	 //ǰ��
	 //TIM5->CCR1 = 4200; //�ҵ��in1,PA0
	 //TIM5->CCR2 = 0; //�ҵ��in2,PA1
	 //TIM3->CCR3 = 0; //����in1,PB0
	 //TIM3->CCR4 = 4200; //����in2,PB1
