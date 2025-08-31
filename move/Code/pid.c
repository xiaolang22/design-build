#include "pid.h"

PID_controller  yaw_v_pid;
PID_controller  yaw_pid;
extern float PID_yaw_v[];
extern float PID_yaw[];
extern float Mechanical_zero[];


void pid_init_yaw_v(void)
{
	  yaw_v_pid.setpoint=Mechanical_zero[1];
    yaw_v_pid.actualvalue=0;
    yaw_v_pid.sumerror=0;
    yaw_v_pid.error=0;
    yaw_v_pid.lasterror=0;
    yaw_v_pid.preverror=0;
    yaw_v_pid.P=PID_yaw_v[0];
    yaw_v_pid.I=PID_yaw_v[1];
    yaw_v_pid.D=PID_yaw_v[2];
}

void pid_init_yaw(void)
{
	  yaw_pid.setpoint=Mechanical_zero[2];
    yaw_pid.actualvalue=0;
    yaw_pid.sumerror=0;
    yaw_pid.error=0;
    yaw_pid.lasterror=0;
    yaw_pid.preverror=0;
    yaw_pid.P=PID_yaw[0];
    yaw_pid.I=PID_yaw[1];
    yaw_pid.D=PID_yaw[2];
}

float increment__yaw_v(PID_controller *PID,float Feedback_value)
{
	  PID->error=(float)(PID->setpoint-Feedback_value);
    PID->sumerror +=PID->error;
    PID->actualvalue =(PID->P*PID->error)
                     +(PID->I*PID->sumerror)
                     +(PID->D*(PID->error-PID->lasterror));
		if (PID->actualvalue > 2000)
        PID->actualvalue = 2000; //===输出限幅
    if (PID->actualvalue < -2000)
        PID->actualvalue = -2000; //===输出限幅
    PID->lasterror=PID->error;
    return PID->actualvalue;
}

float increment__yaw(PID_controller *PID,float Feedback_value)
{
	  PID->error=(float)(PID->setpoint-Feedback_value);
    PID->sumerror +=PID->error;
    PID->actualvalue =(PID->P*PID->error)
                     +(PID->I*PID->sumerror)
                     +(PID->D*(PID->error-PID->lasterror));
		if (PID->actualvalue > 4500)
        PID->actualvalue = 4500; //===输出限幅
    if (PID->actualvalue < -4500)
        PID->actualvalue = -4500; //===输出限幅
    PID->lasterror=PID->error;
    return PID->actualvalue;
}

void Change_yaw_v_setpoint(PID_controller *PID,float setpoint)
{
		PID->setpoint=setpoint;
}

void Change_yaw_setpoint(PID_controller *PID,float setpoint)
{
		PID->setpoint=setpoint;
}

