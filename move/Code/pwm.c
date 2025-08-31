#include "pwm.h"

/*************************************************************
函数功能：PWM赋值
入口参数：motor_left:左电机PWM值，motor_right:右电机PWM值
返回  值：无
*************************************************************/
//in1——pwm，in2——0：正转
//in1——0，in2——pwm：反转
//左轮
void Set_PWM_left(int in1,int in2)
{
	TIM3->CCR3 = in1; //左电机in1,PB0
	TIM3->CCR4 = in2; //左电机in2,PB1	
}

//右轮
void Set_PWM_right(int in1,int in2)
{
	TIM5->CCR1 = in1; //右电机in1,PA0
	TIM5->CCR2 = in2; //右电机in2,PA1
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
