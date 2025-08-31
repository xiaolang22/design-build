#include "moto.h"
#include "PWM.h"

extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //����ٶ�PID����

/**************************************************************************
�������ܣ����������ת
��ڲ�����mode   mode=0ʱΪ��ת  mode=1ʱ��ת
����  ֵ����
**************************************************************************/
void moto(int mode)
{
	if(mode==1)    //��ת
	{
	  PWMA_IN1=7200;PWMA_IN2=5000;
	  PWMB_IN1=7200;PWMB_IN2=5000;
	}
	 if(mode==0)   //����
	{
	  PWMA_IN2=7200;PWMA_IN1=5000;
	  PWMB_IN2=7200;PWMB_IN1=5000;
	 }
}
/***************************************************************************
�������ܣ�����ıջ�����
��ڲ���������ı�����ֵ
����ֵ  �������PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
     int Bias;  //������ر���
		static int ControlVelocity, Last_bias; //��̬�������������ý�������ֵ��Ȼ����
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_bias=Bias;	
		return ControlVelocity; //�����ٶȿ���ֵ
}


int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //������ر���
		static int ControlVelocity, Last_bias; //��̬�������������ý�������ֵ��Ȼ����
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_bias=Bias;	
		return ControlVelocity; //�����ٶȿ���ֵ
}


