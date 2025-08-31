#include "encoder.h"

/**************************************************************************
Function: Read encoder count per unit time
Input   : TIMX：Timer
Output  : none
函数功能：单位时间读取编码器计数
入口参数：TIMX：定时器2/4，两路编码器
返回  值：速度值
**************************************************************************/
/*
int Read_Encoder(uint8_t TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
	     case 2:  Encoder_TIM= TIM2 -> CNT;  TIM2 -> CNT=0;break;	//读取定时器2对应编码器
			 case 4:  Encoder_TIM= TIM4 -> CNT;  TIM4 -> CNT=0;break;	//读取定时器4对应编码器
			 default: Encoder_TIM=0;
		 
	 }
	 
	 if(Encoder_TIM > 200000000) Encoder_TIM = Encoder_TIM - 4294967295;
	    //if(Encoder_TIM<0)   Encoder_TIM=-Encoder_TIM;          //将编码器读到的数据变为正数
		return Encoder_TIM;
}
*/
/*
int32_t Read_Encoder(uint8_t TIMX)
{
    uint32_t raw_count;  // 存储原始无符号计数值
    
    switch(TIMX)
    {
        case 2:  
            raw_count = TIM2->CNT;
            TIM2->CNT = 0;
            break;
        case 4:  
            raw_count = TIM4->CNT;
            TIM4->CNT = 0;
            break;
        default: 
            raw_count = 0;
    }
    
    // 处理反转情况：当计数值大于最大计数值的一半时
    if(raw_count > 0x7FFFFFFF) {  // 0x7FFFFFFF = 2147483647
        // 将无符号值转换为有符号值（处理反转）
        return (int32_t)(raw_count - 0x100000000);  // 0x100000000 = 4294967296
    }
    
    // 正转情况：直接返回计数值
    return (int32_t)raw_count;
}
*/

int Read_Encoder(uint8_t TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
	    case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;	//读取定时器2对应编码器
		 case 1:  Encoder_TIM= (short)TIM1 -> CNT;  TIM1 -> CNT=0;break;	//读取定时器1对应编码器
		 default: Encoder_TIM=0;
		 
	 }
	    //if(Encoder_TIM<0)   Encoder_TIM=-Encoder_TIM;          //将编码器读到的数据变为正数
		return Encoder_TIM;
}
/*
int32_t Read_Encoder(uint8_t TIMX)
{
    uint32_t raw_count;  // 存储原始无符号计数值
    
    switch(TIMX)
    {
        case 2:  
            raw_count = TIM2->CNT;
            TIM2->CNT = 0;
            break;
        case 4:  
            raw_count = TIM4->CNT;
            TIM4->CNT = 0;
            break;
        default: 
            raw_count = 0;
    }
    
    // 处理反转情况：当计数值大于最大计数值的一半时
    if(raw_count > 0x7FFFFFFF) {  // 0x7FFFFFFF = 2147483647
        // 将无符号值转换为有符号值（处理反转）
        return (int32_t)(raw_count - 0x100000000);  // 0x100000000 = 4294967296
    }
    
    // 正转情况：直接返回计数值
    return (int32_t)raw_count;
}*/
