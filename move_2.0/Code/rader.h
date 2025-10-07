#ifndef __RADER_H
#define __RADER_H

#include "main.h"
#include "usart.h"
#include "gpio.h"


//雷达帧相关定义
#define LS_HEADER1      0x0A   //请求报文帧头
#define LS_HEADER2      0x05   //请求报文帧头
#define LS_F_LEN        84   //请求报文帧头


//雷达点结构体定义
typedef struct
{
	uint16_t       angle;     //角度
	uint16_t    distance;     //距离
}LaserPointTypeDef;


void LS_DataHandle(void);
void AX_LASER_Start(void);
void AX_LASER_Stop(void);

#endif
