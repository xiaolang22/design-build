#include "pwm.h"

//in1——pwm，in2——0：正转
//in1——0，in2——pwm：反转
//B轮
void Set_PWM_B(int in1,int in2)
{ 
	TIM3->CCR3 = in1; //B电机in1,PB0
	TIM3->CCR4 = in2; //B电机in2,PB1	
}

//A轮
void Set_PWM_A(int in1,int in2)
{
	TIM5->CCR1 = in1; //A电机in1,PA0
	TIM5->CCR2 = in2; //A电机in2,PA1
}

	 //in1——pwm，in2——0：正转
	 //in1——0，in2——pwm：反转
	 //后退
	 //TIM5->CCR1 = 0; //右电机in1,PA0
	 //TIM5->CCR2 = 4200; //右电机in2,PA1
	 //TIM3->CCR3 = 4200; //左电机in1,PB0
	 //TIM3->CCR4 = 0; //左电机in2,PB1
	 //前进
	 //TIM5->CCR1 = 4200; //右电机in1,PA0
	 //TIM5->CCR2 = 0; //右电机in2,PA1
	 //TIM3->CCR3 = 0; //左电机in1,PB0
	 //TIM3->CCR4 = 4200; //左电机in2,PB1
