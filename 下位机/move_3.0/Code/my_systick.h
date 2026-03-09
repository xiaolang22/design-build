#ifndef __MY_SYSTICK_H
#define __MY_SYSTICK_H

#include "main.h"
#include "tim.h"

uint64_t GetTime_us(void);
uint64_t GetTime_ms(void);
void MY_TIM2_IRQHandler(TIM_HandleTypeDef *htim);	

#endif
