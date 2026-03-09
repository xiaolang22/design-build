#ifndef PID_H
#define PID_H

#include "main.h"
#include "pid.h"

//PID控制器结构
typedef struct
{
    double setpoint;             //目标值
    double actualvalue;          //输出值 
    float P;										//PID参数
    float I;
    float D;
		double sumerror;             //累计误差
    double error;                //error[K]
    double lasterror;            //error[K-1]
    double preverror;            //error[K-2]
}PID_controller;

void PID_init(PID_controller *pid_controller);																	//PID控制器初始化
double PID_increment(PID_controller *pid_controller, double Feedback_value);		//更新PID控制器，并返回输出值
void Change_PID_setpoint(PID_controller *pid_controller, double setpoint);			//改变PID控制器目标值

#endif
