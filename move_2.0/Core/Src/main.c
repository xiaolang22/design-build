/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <math.h>
#include "encoder.h"
#include "my_systick.h"
#include "pwm.h"
#include "rader.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//编码器
//extern int64_t encoderA_line, encoderB_line;	//单轮线数
//extern double wheelA_dis, wheelB_dis;					//单轮距离
double A_speed, B_speed;

//雷达模块
extern uint8_t uart4_rx_con;       //接收计数器
extern uint8_t uart4_rx_chksum;      //异或校验
extern uint8_t uart4_rx_buf[100];     //接收缓冲
extern uint8_t uart4_tx_buf[10];     //接收缓冲
extern uint8_t uart4_rx_data;  // ???????
extern LaserPointTypeDef ax_ls_point[250];
uint8_t time_str[27];

int sent_cnt = 0;

//串口发送模块（蓝牙）
// 添加DMA发送完成标志
volatile uint8_t uart1_tx_dma_complete = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);  // 启动自定义滴答计时器
	
	//pwm模块，开启pwm
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	//Set_PWM_B(0, 5000);
	//Set_PWM_A(5000, 0);
	
	//雷达模块
	HAL_Delay(500); // 等待雷达上电初始化
	AX_LASER_Start();
	
	//所有模块准备就绪，开启任务调度定时器
	HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		char buff[50] ;
//		//sprintf(buff, "A_l:%lld, A_d:%lf; B_l:%lld, B_d:%lf \r\n", ReadLines_A(), ReadDistance_A(), ReadLines_B(), ReadDistance_B());//格式化输出字符串
//		//sprintf(buff, "%lld, %lf, %lld, %lf, %llu\r\n", ReadLines_A(), ReadDistance_A(), ReadLines_B(), ReadDistance_B(), GetTime_us());//格式化输出字符串
//		sprintf(buff, "%lf, %lf, %lf, %lf, %lld, %lld\r\n", GetSpeed_A_filter(), GetSpeed_A(), GetSpeed_B_filter(),GetSpeed_B(), ReadLines_A(), ReadLines_B());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf, %lld, %lld\r\n", GetSpeed_A_filter(), GetSpeed_B_filter(), ReadLines_A(), ReadLines_B());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf\r\n", GetSpeed_A_filter(), GetSpeed_B_filter());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf\r\n", A_speed, B_speed);//格式化输出字符串
//		HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
		
		//发布雷达数据
//		sent_cnt++;
//		for(int i = 0; i < 250; i++)
//		{
//			sprintf(time_str, "frame: %d; point_num:%d; A:%d.%d; D:%d mm\r\n", sent_cnt, i, ax_ls_point[i].angle/100, ax_ls_point[i].angle%100, ax_ls_point[i].distance); 
//			HAL_UART_Transmit_IT(&huart1, time_str, sizeof(time_str));
//			HAL_Delay(500); 
//		}
		
//		sent_cnt++;
//    
//    // 复制雷达数据到临时数组，避免数据竞争
//    LaserPointTypeDef temp_points[250];
//    
//    // 短暂禁用中断确保数据一致性
//    __disable_irq();
//    memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
//    __enable_irq();
//    
//    // 发送帧计数
//    sprintf(time_str, "frame: %d\r\n", sent_cnt); 
//    HAL_UART_Transmit_IT(&huart1, time_str, strlen(time_str));
//    
////    // 发送复制的数据
////    for(int i = 0; i < 250; i++) {
////        sprintf(time_str, "frame: %d; point_num:%d; A:%d.%d; D:%d mm\r\n", 
////                sent_cnt, i, 
////                temp_points[i].angle/100, temp_points[i].angle%100, 
////                temp_points[i].distance); 
////        HAL_UART_Transmit_IT(&huart1, time_str, strlen(time_str));
////    }
//		// 发送复制的数据 - 修改为一次性发送所有数据，并添加分隔符
//		uint16_t data_buffer[1000]; // 250个点 × (3个数据元素 + 1个分隔符) = 1000个元素

//		// 将结构体数据重新组织到整型数组中，并添加分隔符
//		for(int i = 0; i < 250; i++) {
//				data_buffer[4*i]     = temp_points[i].angle / 100;    // 角度整数部分
//				data_buffer[4*i + 1] = temp_points[i].angle % 100;    // 角度小数部分
//				data_buffer[4*i + 2] = temp_points[i].distance;       // 距离值
//				data_buffer[4*i + 3] = 0xFFFF;                        // 分隔符 -1 (用0xFFFF表示)
//		}

//		// 一次性发送所有数据
//		HAL_UART_Transmit(&huart1, (uint8_t*)data_buffer, sizeof(data_buffer), 1000);
		
		//HAL_Delay(100);
		
//		sent_cnt++;
//    
//    // 复制雷达数据到临时数组，避免数据竞争
//    LaserPointTypeDef temp_points[250];
//    
//    // 短暂禁用中断确保数据一致性
//    __disable_irq();
//    memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
//    __enable_irq();
//    
//    // 创建足够大的ASCII缓冲区
//    char ascii_buffer[10000]; // 预估大小，每个点约40字符
//    int offset = 0;
//    
//    // 添加帧头
//    offset += sprintf(ascii_buffer + offset, "frame:%d\n", sent_cnt);
//    
//    // 添加数据点，每行一个点
//    for(int i = 0; i < 250; i++) {
//        offset += sprintf(ascii_buffer + offset, "%d.%02d,%d\n", 
//                         temp_points[i].angle / 100,
//                         temp_points[i].angle % 100,
//                         temp_points[i].distance);
//    }
//    
//    // 添加帧尾分隔符
//    offset += sprintf(ascii_buffer + offset, "---\n");
//    
//    // 一次性发送所有ASCII数据
//    HAL_UART_Transmit(&huart1, (uint8_t*)ascii_buffer, offset, 1000);
//    
//    HAL_Delay(100); // 控制发送频率

		// 等待上一次DMA发送完成
    if (uart1_tx_dma_complete) {
        sent_cnt++;
        
        // 复制雷达数据到临时数组，避免数据竞争
        LaserPointTypeDef temp_points[250];
        
        // 短暂禁用中断确保数据一致性
        __disable_irq();
        memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
        __enable_irq();
        
        // 创建足够大的ASCII缓冲区
        static char ascii_buffer[10000]; // 使用静态数组避免栈溢出
        int offset = 0;
        
        // 添加帧头
        offset += sprintf(ascii_buffer + offset, "frame:%d\n", sent_cnt);
        
        // 添加数据点，每行一个点
        for(int i = 0; i < 250; i++) {
            offset += sprintf(ascii_buffer + offset, "%d.%02d,%d\n", 
                             temp_points[i].angle / 100,
                             temp_points[i].angle % 100,
                             temp_points[i].distance);
        }
        
        // 添加帧尾分隔符
        offset += sprintf(ascii_buffer + offset, "---\n");
        
        // 设置发送完成标志为0（正在发送）
        uart1_tx_dma_complete = 0;
        
        // 使用DMA发送所有ASCII数据
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ascii_buffer, offset);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//*****************************DMA中断**********************************//
// DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        uart1_tx_dma_complete = 1;  // 清除忙标志
    }
}

// DMA发送错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        uart1_tx_dma_complete = 1;  // 清除忙标志
    }
}

//*****************************任务调度**********************************//
int count1;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim == &htim4) 
		{
			count1++;//分时器
			
//			A_speed = GetSpeed_A_filter();
//			B_speed = GetSpeed_B_filter();
		
		
		}	
}

//*****************************串口通信**********************************//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	// 雷达数据接收区
	if (huart->Instance == USART6)
    {
        uint8_t Res = uart4_rx_data;
        uint8_t temp;
			
        if (uart4_rx_con < 3)
        {
            if(uart4_rx_con == 0)  //接收帧头1 
            {
                //判断帧头1
                if((Res>>4) == LS_HEADER1)
                {
                    uart4_rx_buf[uart4_rx_con] = Res;
                    uart4_rx_con = 1;                    
                }
            }
            else if(uart4_rx_con == 1) //接收帧头2
            {
                //判断帧头2
                if((Res>>4) == LS_HEADER2)
                {
                    uart4_rx_buf[uart4_rx_con] = Res;
                    uart4_rx_con = 2;
                }
                else
                {
                    uart4_rx_con = 0;                        
                }                
            }
            else  //接收第一个数据
            {
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_con = 3;
                
                //赋值校验
                uart4_rx_chksum = Res;	
            }
        }			
        else  //接收数据
        {
            //判断是否接收完
            if(uart4_rx_con < (LS_F_LEN-1))
            {
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_con++;
                uart4_rx_chksum = uart4_rx_chksum^Res;
            }
            else
            {
                //接收最后一个数据
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_chksum = uart4_rx_chksum^Res;
                
                //复位
                uart4_rx_con = 0;
                
                //计算传输的校验数据
                temp = ((uint8_t)(uart4_rx_buf[1]<<4)) + (uint8_t)(uart4_rx_buf[0]&0x0F);
                
                //判断校验是否正确
                if( uart4_rx_chksum == temp)
                {
                    //接收完毕，进行帧数据处理
                    LS_DataHandle();	
                }
            }
        }
				//重新开启中断
        HAL_UART_Receive_IT(&huart6, &uart4_rx_data, 1);
    }
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
