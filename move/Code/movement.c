#include "movement.h"

extern int in1_left, in1_right, in2_left, in2_right;//双轮pwm控制量
extern uint8_t zhaunxiang_stop_flag;//转向环标志位

//停止
void stop(void)
{
	in1_left = 0;
	in2_left = 0;
	in1_right = 0;
	in2_right = 0;
	zhaunxiang_stop_flag = 0;
}

//前进
void forward(void)
{
	in1_right = 5000;
	in2_right = 0;
	in1_left = 0;
	in2_left = 5000;
	zhaunxiang_stop_flag = 0;
}

//后退
void back(void)
{
	in1_right = 0;
	in2_right = 5000;
	in1_left = 5000;
	in2_left = 0;
	zhaunxiang_stop_flag = 0;
}

//左转
void left(void)
{
	in1_right = 4200;
	in2_right = 0;
	in1_left = 4200;
	in2_left = 0;
	zhaunxiang_stop_flag = 1;
}

//右转
void right(void)
{
	in1_right = 0;
	in2_right = 4500;
	in1_left = 0;
	in2_left = 4500;
	zhaunxiang_stop_flag = 1;
}


