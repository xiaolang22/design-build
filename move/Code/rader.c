#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "rader.h"

uint8_t uart4_rx_con=0;       //接收计数器
uint8_t uart4_rx_chksum;      //异或校验
uint8_t uart4_rx_buf[100];     //接收缓冲
uint8_t uart4_tx_buf[10];     //接收缓冲


uint8_t uart4_rx_data = 0;  // ???????

//扫描一圈的雷达数据

LaserPointTypeDef ax_ls_point[250];

/**
  * @简  述  数据处理函数
  * @参  数  无
  * @返回值	 无
  */
void LS_DataHandle(void)
{

	uint8_t i;
	float temp;
	
	static uint16_t cnt = 0;
	
	static float angle_last = 0;
	
	//每秒采集5000次10HZ，转一圈采集500个点，方便单片机处理，2个点取一个点
	static LaserPointTypeDef point[250];
	

		    // ?????????????
//    char temp_str[50];
//    HAL_UART_Transmit(&huart1, (uint8_t*)"RX Buffer: ", 11, 100);  // ????

//    for (int i = 0; i < 10; i++) {
//        // ????????????
//        sprintf(temp_str, "%02X ", uart4_rx_buf[i]);
//        HAL_UART_Transmit(&huart1, (uint8_t*)temp_str, strlen(temp_str), 100);  // ???????
//    }
//    // ???
//    HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, 100);
		
		
	float angle_new = (((uint16_t)((uart4_rx_buf[3]&0x7F)<<8)) + uart4_rx_buf[2])/64.0;
	float angle_area;
		
		
//    sprintf(temp_str, "uart4_rx_buf[2]: %02X, uart4_rx_buf[3]: %02X, angle_new: %.2f\n", uart4_rx_buf[2], uart4_rx_buf[3], angle_new);
//    HAL_UART_Transmit(&huart1, (uint8_t*)temp_str, strlen(temp_str), 100);

    

	
	//起始角度大于结束角度，跨过360°
	if(angle_new > angle_last)
	{
		angle_area  = (angle_new - angle_last)/20;
		
		for(i=0; i<20; i++)
		{
			temp = angle_new + angle_area*i;
			
			//计算角度
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}
			
			//计算距离
			point[cnt+i].distance =  ((uint16_t)(uart4_rx_buf[5+i*4]<<8)) + (uint8_t)uart4_rx_buf[4+i*4];

		}
	}
	else
	{
		angle_area = (angle_new + 360 - angle_last)/20;
		
		for(i=0; i<20; i++)
		{
		
			temp = angle_new + angle_area*i;

			
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}
			
			//计算距离
			point[cnt+i].distance =  ((uint16_t)(uart4_rx_buf[5+i*4]<<8)) + (uint8_t)uart4_rx_buf[4+i*4];
		}
		
	}

	//赋值上一次测量角度
	angle_last = angle_new;	
	
	//输出调试数据
	//printf("%d %d %d \r\n",cnt, point[0].angle, point[0].distance);	

	//一帧数据解析结束
	cnt = cnt+20;
	
	//判断是否转弯一圈（雷达转一圈有250个点）
	  
	if(cnt > 260)
	{
		//将数组的数据转移到外部数组中，避免覆盖数据
		for(i=0; i<250; i++)
		{
			//计算角度
			ax_ls_point[i].angle = point[i].angle;
			ax_ls_point[i].distance = point[i].distance;
		}
		
		//复位
		cnt = 0;
	}

}


/**
  * @简  述  雷达启动（密实模式）
  * @参  数  无
  * @返回值	 无
  */
void AX_LASER_Start(void)   
{
	uint8_t i;	
	
	uart4_tx_buf[0] = 0xA5;  //帧头
	uart4_tx_buf[1] = 0x82;  //启动扫描命令
	uart4_tx_buf[2] = 05;    
	uart4_tx_buf[3] = 0;    
	uart4_tx_buf[4] = 0;    
	uart4_tx_buf[5] = 0;    
	uart4_tx_buf[6] = 0;    
	uart4_tx_buf[7] = 0;    
	uart4_tx_buf[8] = 0x22;  //校验和
	

	if (HAL_UART_Receive_IT(&huart6, &uart4_rx_data, 1) != HAL_OK) {
      Error_Handler();
  }
  	HAL_Delay(2000);
	
	//查询传输方式
	for(i=0; i<9; i++)
	{
		HAL_UART_Transmit(&huart6, &uart4_tx_buf[i], 1, 100);
		while (HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
		
		HAL_UART_Transmit(&huart1,"one bit sent", 12, 100);

	}
}


/**
  * @简  述  雷达关闭
  * @参  数  无
  * @返回值	 无
  */
void AX_LASER_Stop(void)   
{
	uint8_t i;	

	uart4_tx_buf[0] = 0xA5;       //帧头
	uart4_tx_buf[1] = 0x25;       //关闭命令
	uart4_tx_buf[2] = 0xA5+0x25;  //校验和
	
	//查询传输方式
	for(i=0; i<3; i++)
	{
//		USART_SendData(UART4, uart4_tx_buf[i]);
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TC) != SET);
		
		
		HAL_UART_Transmit(&huart6, &uart4_tx_buf[i], 1, 100);
		while (HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
	}
}

