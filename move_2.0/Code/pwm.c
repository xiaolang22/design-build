#include "pwm.h"

//in1����pwm��in2����0����ת
//in1����0��in2����pwm����ת
//B��
void Set_PWM_B(int in1,int in2)
{ 
	TIM3->CCR3 = in1; //B���in1,PB0
	TIM3->CCR4 = in2; //B���in2,PB1	
}

//A��
void Set_PWM_A(int in1,int in2)
{
	TIM5->CCR1 = in1; //A���in1,PA0
	TIM5->CCR2 = in2; //A���in2,PA1
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
