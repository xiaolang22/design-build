/*
	实现了双轮速度环和偏航角速度环
	
	双轮速度环：
		双轮速度环为分段式PID，针对不同目标速度设置不同PID参数和积分限幅
		注意，一般实际速度最大值为0.85左右，设置0.9的目标速度存在静差，若后期需要用到0.85以上的速度，需要再加一组pid参数
		
*/

#include "pid.h"



//PID控制器
PID_controller A_speed_PID_controller;	//A轮速度PID控制器
PID_controller B_speed_PID_controller;	//B轮速度PID控制器
PID_controller gz_PID_controller;				//偏航角速度PID控制器

//机械零点（默认初始目标）
float Mechanical_zero[] = {0,0,0};//A轮速度  B轮速度  偏航角速度

//PID参数
float A_speed_low_PID_param[] = {12000, 1000, 8000};		//P I D  {25000, 10000, 0}
float A_speed_mid_PID_param[] = {15000, 5000, 8000};		//P I D  {25000, 10000, 0}
float A_speed_high_PID_param[] = {24000, 2000, 0};		//P I D  {25000, 10000, 0}

float B_speed_low_PID_param[] = {12000, 1000, 5000};		//P I D  {25000, 10000, 0}
float B_speed_mid_PID_param[] = {10000, 9000, 2000};		//P I D  {25000, 10000, 0}
float B_speed_high_PID_param[] = {12000, 1000, 1000};		//P I D  {25000, 10000, 0}

float gz_low_PID_param[]			= {-0.001, -0.00012, -0.00001};		//P I D  {-0.0006, 0, -0.0005};	  {-0.0007, -0.001, -0.0027}100有点抖但还好  {-0.0006, -0.001, -0.002}
//float gz_high_PID_param[]			= {-0.002, 0, 0};	
//积分限幅上下限
float sum_bound_A_up = 0, sum_bound_A_down = 0;
float sum_bound_B_up = 0, sum_bound_B_down = 0;
float sum_bound_gz_up = 0, sum_bound_gz_down = 0;


//PID控制器初始化
void PID_init(PID_controller *pid_controller)
{
	//不同PID控制器的个性化参数
	if(pid_controller == &A_speed_PID_controller)	//A轮速度PID控制器
	{
		pid_controller->setpoint = Mechanical_zero[0];//目标值
		pid_controller->P = 0;//PID参数
		pid_controller->I = 0;
		pid_controller->D = 0;
	}
	if(pid_controller == &B_speed_PID_controller)	//B轮速度PID控制器
	{
		pid_controller->setpoint = Mechanical_zero[1];//目标值
		pid_controller->P = 0;//PID参数
		pid_controller->I = 0;
		pid_controller->D = 0;
	}
	if(pid_controller == &gz_PID_controller)	//偏航角速度PID控制器
	{
		pid_controller->setpoint = Mechanical_zero[2];//目标值
		pid_controller->P = 0;//PID参数
		pid_controller->I = 0;
		pid_controller->D = 0;
	}
	
	//所有PID控制器的共同初始化参数
	pid_controller->actualvalue = 0;	//输出值 
	pid_controller->sumerror = 0;			//累计偏差
	pid_controller->error = 0;				//error[K]
	pid_controller->lasterror = 0;		//error[K-1]
	pid_controller->preverror = 0;		//error[K-2]
}

//更新PID控制器，并返回输出值
double PID_increment(PID_controller *pid_controller, double Feedback_value)
{
		//计算error和累计error
	  pid_controller->error = (double)(pid_controller->setpoint - Feedback_value);	//更新error[K]
    pid_controller->sumerror += pid_controller->error;													//更新累积误差
		
		//（各异）
		//********A轮速度PID控制器
		if(pid_controller == &A_speed_PID_controller)	
		{
			////////根据目标值选择不同参数////////
			if(pid_controller->setpoint < 0.3 && pid_controller->setpoint > -0.3)	//低速
			{
				//更新PID参数
				pid_controller->P = A_speed_low_PID_param[0];//PID参数（低速）
				pid_controller->I = A_speed_low_PID_param[1];
				pid_controller->D = A_speed_low_PID_param[2];
				
				//更新积分限幅
				sum_bound_A_up = 1500;
				sum_bound_A_down = -1500;	
			}
			else if((pid_controller->setpoint >= 0.3 && pid_controller->setpoint <= 0.5) || (pid_controller->setpoint <= -0.3 && pid_controller->setpoint >= -0.5))	//中速
			{
				//更新PID参数
				pid_controller->P = A_speed_mid_PID_param[0];//PID参数（中速）
				pid_controller->I = A_speed_mid_PID_param[1];
				pid_controller->D = A_speed_mid_PID_param[2];
				
				//更新积分限幅
				sum_bound_A_up = 1500;
				sum_bound_A_down = -1500;	
			}
			else if((pid_controller->setpoint > 0.5 && pid_controller->setpoint <= 1) || (pid_controller->setpoint < -0.5 && pid_controller->setpoint >= -1))	//高速
			{
				//更新PID参数
				pid_controller->P = A_speed_high_PID_param[0];//PID参数（高速）
				pid_controller->I = A_speed_high_PID_param[1];
				pid_controller->D = A_speed_high_PID_param[2];
				
				//更新积分限幅
				sum_bound_A_up = 3000;
				sum_bound_A_down = -3000;	
			}
			
			////////根据参数计算输出////////
			//根据参数计算三个量
			double p_actualvalue = pid_controller->P * pid_controller->error;
			double i_actualvalue = pid_controller->I * pid_controller->sumerror;
			double d_actualvalue = pid_controller->D * (pid_controller->error - pid_controller->lasterror);
			
			pid_controller->lasterror = pid_controller->error;			//计算完毕，更新error[K-1]
			
			//积分限幅
			if (i_actualvalue > sum_bound_A_up)
        i_actualvalue = sum_bound_A_up;
			if (i_actualvalue < sum_bound_A_down)
        i_actualvalue = sum_bound_A_down; 
			
			//计算输出值
			pid_controller->actualvalue = p_actualvalue + i_actualvalue + d_actualvalue;
			
			//输出限幅
			if (pid_controller->actualvalue > 4000)
        pid_controller->actualvalue = 4000;
			if (pid_controller->actualvalue < -4000)
        pid_controller->actualvalue = -4000; 
		}	
		

		//********B轮速度PID控制器
		if(pid_controller == &B_speed_PID_controller)	
		{
			////////根据目标值选择不同参数////////
			if(pid_controller->setpoint <= 0.2 && pid_controller->setpoint >= -0.2)	//低速
			{
				//更新PID参数
				pid_controller->P = B_speed_low_PID_param[0];//PID参数（低速）
				pid_controller->I = B_speed_low_PID_param[1];
				pid_controller->D = B_speed_low_PID_param[2];
				
				//更新积分限幅
				sum_bound_B_up = 300;
				sum_bound_B_down = -300;	
			}
			else if((pid_controller->setpoint > 0.2 && pid_controller->setpoint <= 0.45) || (pid_controller->setpoint < -0.2 && pid_controller->setpoint >= -0.45))	//中速
			{
				//更新PID参数
				pid_controller->P = B_speed_mid_PID_param[0];//PID参数（中速）
				pid_controller->I = B_speed_mid_PID_param[1];
				pid_controller->D = B_speed_mid_PID_param[2];
				
				//更新积分限幅
				sum_bound_B_up = 1500;
				sum_bound_B_down = -1500;	
			}
			else if((pid_controller->setpoint > 0.45 && pid_controller->setpoint <= 1) || (pid_controller->setpoint < -0.45 && pid_controller->setpoint >= -1))	//高速
			{
				//更新PID参数
				pid_controller->P = B_speed_high_PID_param[0];//PID参数（高速）
				pid_controller->I = B_speed_high_PID_param[1];
				pid_controller->D = B_speed_high_PID_param[2];
				
				//更新积分限幅
				sum_bound_B_up = 3500;
				sum_bound_B_down = -3500;	
			}
			
			////////根据参数计算输出////////
			//根据参数计算三个量
			double p_actualvalue = pid_controller->P * pid_controller->error;
			double i_actualvalue = pid_controller->I * pid_controller->sumerror;
			double d_actualvalue = pid_controller->D * (pid_controller->error - pid_controller->lasterror);
			
			pid_controller->lasterror = pid_controller->error;			//计算完毕，更新error[K-1]
			
			//积分限幅
			if (i_actualvalue > sum_bound_B_up)
        i_actualvalue = sum_bound_B_up;
			if (i_actualvalue < sum_bound_B_down)
        i_actualvalue = sum_bound_B_down; 
			
			//计算输出值
			pid_controller->actualvalue = p_actualvalue + i_actualvalue + d_actualvalue;
			
			//输出限幅
			if (pid_controller->actualvalue > 4000)
        pid_controller->actualvalue = 4000;
			if (pid_controller->actualvalue < -4000)
        pid_controller->actualvalue = -4000; 
		}	
		
		//********偏航角速度PID控制器
		if(pid_controller == &gz_PID_controller)	
		{
			////////根据目标值选择不同参数////////
			if(pid_controller->setpoint <= 150 && pid_controller->setpoint >= -150)	//低速
			{
				//更新PID参数
				pid_controller->P = gz_low_PID_param[0];//PID参数（低速）
				pid_controller->I = gz_low_PID_param[1];
				pid_controller->D = gz_low_PID_param[2];
				
				//更新积分限幅
				sum_bound_gz_up = 0.2;
				sum_bound_gz_down = -0.2;	
			}
//			else if((pid_controller->setpoint > 70 && pid_controller->setpoint <= 150) || (pid_controller->setpoint < -70 && pid_controller->setpoint >= -150))	//高速
//			{
//				//更新PID参数
//				pid_controller->P = gz_high_PID_param[0];//PID参数（高速）
//				pid_controller->I = gz_high_PID_param[1];
//				pid_controller->D = gz_high_PID_param[2];
//				
//				//更新积分限幅
//				sum_bound_gz_up = 0.15;
//				sum_bound_gz_down = -0.15;	
//			}
			
			////////根据参数计算输出////////
			//根据参数计算三个量
			double p_actualvalue = pid_controller->P * pid_controller->error;
			double i_actualvalue = pid_controller->I * pid_controller->sumerror;
			double d_actualvalue = pid_controller->D * (pid_controller->error - pid_controller->lasterror);
			
			pid_controller->lasterror = pid_controller->error;			//计算完毕，更新error[K-1]
			
			//积分限幅
			if (i_actualvalue > sum_bound_gz_up)
        i_actualvalue = sum_bound_gz_up;
			if (i_actualvalue < sum_bound_gz_down)
        i_actualvalue = sum_bound_gz_down;
			
			//计算输出值
			pid_controller->actualvalue = p_actualvalue + i_actualvalue + d_actualvalue;
			
			//输出限幅
			if (pid_controller->actualvalue > 0.7)
        pid_controller->actualvalue = 0.7;
			if (pid_controller->actualvalue < -0.7)
        pid_controller->actualvalue = -0.7; 
		}	
		
		//返回最终计算得到的输出值
    return pid_controller->actualvalue;
}

//改变PID控制器目标值
void Change_PID_setpoint(PID_controller *pid_controller, double setpoint)
{
		pid_controller->setpoint = setpoint;
}


