#ifndef PID_H
#define PID_H

#include "main.h"
#include "pid.h"

typedef struct
{
    float setpoint;             //Ŀ��ֵ
    float actualvalue;          //����ֵ
    float sumerror;             //�ۼ�ƫ��
    float P;
    float I;
    float D;
    float error;                //error[K]
    float lasterror;            //error[K-1]
    float preverror;            //error[K-2]

}PID_controller;

void pid_init_yaw_v(void);
void pid_init_yaw(void);

float increment__yaw_v(PID_controller *PID,float Feedback_value);
float increment__yaw(PID_controller *PID,float Feedback_value);

void Change_yaw_v_setpoint(PID_controller *PID,float Feedback_value);
void Change_yaw_setpoint(PID_controller *PID,float Feedback_value);

#endif
