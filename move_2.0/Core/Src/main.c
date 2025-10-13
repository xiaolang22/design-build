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
#include "i2c.h"
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
#include "rader.h"
#include "mpu6500.h"
#include "pwm.h"
#include "frame.h"
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

//IMU模块
float ax, ay, az, gx, gy, gz;			//原始数据
float gz_last;										//偏航角计算，用于gz一阶互补滤波
float yaw_last;										//偏航角计算，用于yaw角限幅滤波，消除累积误差
uint32_t time_latest = 0;					//偏航角计算，用于计算积分dt
uint32_t time_last = 0;
float roll, pitch, yaw;						//欧拉角解析结果（转存变量float）
extern Attitude_t attitude;				//欧拉角解析结果（mpu6500文件）

//雷达模块
extern uint8_t uart4_rx_con;       //接收计数器
extern uint8_t uart4_rx_chksum;      //异或校验
extern uint8_t uart4_rx_buf[100];     //接收缓冲
extern uint8_t uart4_tx_buf[10];     //接收缓冲
extern uint8_t uart4_rx_data;  
extern LaserPointTypeDef ax_ls_point[250];	//雷达点云数组（结构体数组，包含角度、距离）
uint8_t time_str[50];

//传感器数据帧通信模块
Frame frame;


int sent_cnt = 0;//调试，统计雷达发送帧数

//串口发送模块（蓝牙）
volatile uint8_t uart1_tx_dma_complete = 1;	// 添加DMA发送完成标志

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	//***************************自定义初始化***************************//
	
	//编码器模块（滴答计时器）
	HAL_TIM_Base_Start_IT(&htim2);  // 启动自定义滴答计时器
	
	//PWM模块，开启pwm
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	//Set_PWM_B(0, 5000);
	//Set_PWM_A(5000, 0);
	
	
	//IMU模块
	while(1)//mpu6500初始化
	{
		if(MPU6500_Init(&hi2c1) == 0)
		{
			char buff[50] ;
			sprintf(buff, "mpu6500 init error!");
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
		}
		else
			break;
	}
	
	//雷达模块
	HAL_Delay(500); 	// 等待雷达上电初始化
	AX_LASER_Start();	// 发送请求报文，雷达开始发送应答报文
	
	//传感器数据帧通信模块
	//完成固定数据填充
	frame.header = swap_uint16(FRAME_HEADER);  // 转换为大端序
	frame.point_count = swap_uint16(LASER_DATA_POINTS);
	frame.angle_resolution = swap_float(LASER_ANGLE_RESOLUTION);
	frame.tail = swap_uint16(FRAME_TAIL);
	
	
	//所有模块准备就绪，开启任务调度定时器
	HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		////////////////////串口发送调试区//////////////////
//		char buff[50] ;
//		//sprintf(buff, "A_l:%lld, A_d:%lf; B_l:%lld, B_d:%lf \r\n", ReadLines_A(), ReadDistance_A(), ReadLines_B(), ReadDistance_B());//格式化输出字符串
//		//sprintf(buff, "%lld, %lf, %lld, %lf, %llu\r\n", ReadLines_A(), ReadDistance_A(), ReadLines_B(), ReadDistance_B(), GetTime_us());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf, %lf, %lf, %lld, %lld\r\n", GetSpeed_A_filter(), GetSpeed_A(), GetSpeed_B_filter(),GetSpeed_B(), ReadLines_A(), ReadLines_B());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf, %lld, %lld\r\n", GetSpeed_A_filter(), GetSpeed_B_filter(), ReadLines_A(), ReadLines_B());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf\r\n", GetSpeed_A_filter(), GetSpeed_B_filter());//格式化输出字符串
//		//sprintf(buff, "%lf, %lf\r\n", A_speed, B_speed);//格式化输出字符串
//		//sprintf(buff, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", ax,ay,az,gx,gy,gz);//imu原始数据
//		sprintf(buff, "%d,%d,%d\r\n", roll_int / 100,pitch_int / 100,yaw_int / 100);//imu欧拉角数据
//		
//		HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
		
//		//发布雷达数据
//		sent_cnt++;
//		LaserPointTypeDef temp_points[250];	// 复制雷达数据到临时数组，避免数据竞争	
//		__disable_irq();// 短暂禁用中断确保数据一致性
//		memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
//		__enable_irq();
//		for(int i = 0; i < 250; i++)
//		{
//			sprintf(time_str, "frame: %d; point_num:%d; A:%d.%d; D:%d mm\r\n", sent_cnt, i, temp_points[i].angle/100, temp_points[i].angle%100, temp_points[i].distance); 
//			HAL_UART_Transmit_IT(&huart1, time_str, sizeof(time_str));
//			//HAL_Delay(500); 
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
		

//////////////////DMA串口发送雷达数据帧到上位机///////////////////////////
//		// 等待上一次DMA发送完成
//    if (uart1_tx_dma_complete) {
//        sent_cnt++;
//        
//        // 复制雷达数据到临时数组，避免数据竞争
//        LaserPointTypeDef temp_points[250];
//        
//        // 短暂禁用中断确保数据一致性
//        __disable_irq();
//        memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
//        __enable_irq();
//        
//				// 填充距离数据并计算校验和
//        uint16_t calc_checksum = 0;
//        for(int i = 0; i < LASER_DATA_POINTS; i++) {
//						laser_frame.distances[i] = swap_uint16(temp_points[i].distance);  // 转换为大端序
//            //calc_checksum += temp_points[i].distance;
//						calc_checksum += laser_frame.distances[i];
//        }
//        
////        // 添加其他字段到校验和
////        calc_checksum += laser_frame.point_count;
////        calc_checksum += (uint16_t)(laser_frame.angle_resolution * 100); // 将浮点数转换为整数参与校验
//        
//				laser_frame.checksum = swap_uint16(calc_checksum);  // 转换为大端序
//        
//        // 设置发送完成标志为0（正在发送）
//        uart1_tx_dma_complete = 0;
//        
//        // 使用DMA发送二进制数据帧
//        //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&laser_frame, sizeof(LaserFrame_t));
//				HAL_UART_Transmit_IT(&huart1, (uint8_t*)&laser_frame, sizeof(LaserFrame_t));
//    }


		
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
int count1 = 0;//分时器
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim == &htim4) // 任务调度计时器
		{
			count1++;//分时器
			
			//////////////////////////传感器数据获取(1)：编码器距离//////////////////////////////////////////
			//编码器距离数据获取
			double dis_A = ReadDistance_A();
			double dis_B = ReadDistance_B();
			
      if(uart1_tx_dma_complete == 1)	 //DMA空闲
			{
				/////////////////////////传感器数据获取(2)：imu//////////////////////////////////////////
				//imu数据获取(占用DMA和总线资源)
				//读取原始数据
				MPU6500_Read_Accel(&hi2c1, &ax, &ay, &az);//与DMA发送冲突，移出去会卡死
				MPU6500_Read_Gyro(&hi2c1, &gx, &gy, &gz);	//与DMA发送冲突，移出去会卡死
				//更新两次测量的时间窗口
				time_last = time_latest;
				time_latest = HAL_GetTick();//ms
				float dt = (time_latest - time_last) / 1000.f;	//s
				//对原始三轴角速度（gz）处理：处理零偏和抖动问题
				gz = gz +0.5;
				if(gz <= 1.4 && gz >= -1.4) gz = 0;
				gz = gz_last*0.3 + gz*0.7;
				gz_last = gz;
				//更新偏航角
				Yaw_Update(gz, yaw_last, dt);
				//覆盖变量
				roll = attitude.roll;		//保持初始值0
				pitch = attitude.pitch;	//保持初始值0
				yaw = attitude.yaw;			//积分处理
				
				////////////////DMA串口发送imu、编码器、雷达数据帧到上位机///////////////////////////
				////装载数据帧
				
				uint16_t calc_checksum = 0;	//校验和
				
				//装载IMU数据帧
				frame.imu_raw_data[0] = swap_float(ax);		//原始数据，转换为大端字节序
				frame.imu_raw_data[1] = swap_float(ay);
				frame.imu_raw_data[2] = swap_float(az);
				frame.imu_raw_data[3] = swap_float(gx);
				frame.imu_raw_data[4] = swap_float(gy);
				frame.imu_raw_data[5] = swap_float(gz);
				
				frame.imu_rpy_data[0] = swap_float(roll);	//欧拉角数据，转换为大端字节序
				frame.imu_rpy_data[1] = swap_float(pitch);
				frame.imu_rpy_data[2] = swap_float(yaw);
				
				for(int i = 0; i < IMU_RAW_DATA_LENGTH; i++)	//计算原始数据校验和
				{
					calc_checksum += (uint16_t)(frame.imu_raw_data[i] * 100);	//浮点数特殊处理
				}
				for(int i = 0; i < IMU_RPY_DATA_LENGTH; i++)	//计算欧拉角数据校验和
				{
					calc_checksum += (uint16_t)(frame.imu_rpy_data[i] * 100);	//浮点数特殊处理
				}
				
				//装载编码器距离数据帧
				frame.wheelA_dis = swap_double(dis_A);	//转换为大端字节序
				frame.wheelB_dis = swap_double(dis_B);
				
				calc_checksum += (uint16_t)(frame.wheelA_dis * 100);	//计算校验和
				calc_checksum += (uint16_t)(frame.wheelB_dis * 100);
				
				//装载雷达数据帧
				LaserPointTypeDef temp_points[250];	// 复制雷达数据到临时数组，避免数据竞争	
				__disable_irq();// 短暂禁用中断确保数据一致性
				memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
				__enable_irq();
				
				for(int i = 0; i < LASER_DATA_POINTS - 1; i++) // 雷达数据按角度从小到大排序
				{
						for(int j = 0; j < LASER_DATA_POINTS - i - 1; j++) 
						{
								if(temp_points[j].angle > temp_points[j+1].angle) 
								{
										// 交换两个点的位置
										LaserPointTypeDef temp = temp_points[j];
										temp_points[j] = temp_points[j+1];
										temp_points[j+1] = temp;
								}
						}
				}
					
				for(int i = 0; i < LASER_DATA_POINTS; i++) // 填充雷达距离数据 并计算校验和
				{
						frame.distances[i] = swap_uint16(temp_points[i].distance);  // 转换为大端序
						calc_checksum += frame.distances[i];			//计算校验和
				}
				
//        // 添加其他字段到校验和
//        calc_checksum += laser_frame.point_count;
//        calc_checksum += (uint16_t)(laser_frame.angle_resolution * 100); // 将浮点数转换为整数参与校验
				
				//装载校验和
				frame.checksum = swap_uint16(calc_checksum);  // 转换为大端序
				
				////发送数据帧
				uart1_tx_dma_complete = 0;// 设置发送完成标志为0（正在发送）
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&frame, sizeof(Frame));	// 使用DMA发送二进制数据帧
			}
		}	
}

//*****************************串口通信**********************************//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	/////////////////////////传感器数据获取(3)：雷达//////////////////////////////////////////
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
