/*
	本文件用于实现微秒级滴答计时器，精确计量从上电到某一时间点的时间。
*/

#include "my_systick.h"

#define ARR 4294967295
#define TIMER_CLOCK_MHZ 84

volatile uint64_t period_num = 0;	//周期数

// 获取距离上电的当前时间（微秒）
uint64_t GetTime_us(void)
{
	uint32_t cnt1_tem, cnt2_tem;	//计数器值
	uint64_t period_num_tem;	//周期数
	
	//获取定时器参数
	do
	{
		cnt1_tem = __HAL_TIM_GET_COUNTER(&htim2);
		period_num_tem = period_num;
		cnt2_tem = __HAL_TIM_GET_COUNTER(&htim2);
	}while(cnt2_tem < cnt1_tem);	//异常情况，第二次cnt小于第一次cnt，说明上述读取过程中发生了更新中断。若发生，则再读一次
	
	return (period_num_tem*(ARR+1) + cnt1_tem)/TIMER_CLOCK_MHZ;
}

// 获取距离上电的当前时间（毫秒）
uint64_t GetTime_ms(void)
{
	return GetTime_us()/1000;
}


//	自定义TIM2定时器中断处理函数，ISR直接映射到这，不映射到HAL库的IRQHandle函数
void MY_TIM2_IRQHandler(TIM_HandleTypeDef *htim)
{
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);	//清除标志位
		
		period_num++;	//真正逻辑：更新周期
}
	