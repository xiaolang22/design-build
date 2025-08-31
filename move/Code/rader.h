#ifndef __RADER_H
#define __RADER_H

#include "main.h"
#include "usart.h"
#include "gpio.h"


//�״�֡��ض���
#define LS_HEADER1      0x0A   //������֡ͷ
#define LS_HEADER2      0x05   //������֡ͷ
#define LS_F_LEN        84   //������֡ͷ


//�״��ṹ�嶨��
typedef struct
{
	uint16_t       angle;     //�Ƕ�
	uint16_t    distance;     //����
}LaserPointTypeDef;


void LS_DataHandle(void);
void AX_LASER_Start(void);
void AX_LASER_Stop(void);

#endif
