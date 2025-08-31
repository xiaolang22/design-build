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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "string.h"
#include <stdio.h>
#include <math.h>
#include "mpu6500.h"
#include "pwm.h"
#include "pid.h"
#include "movement.h"
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
	//编码器变量
	uint32_t systick = 0;
	int encoder_A_l = 0,encoder_B_l = 0;//编码器（上一次）
	int encoder_A_n = 0,encoder_B_n = 0;//编码器（这一次）
	
	
	//imu变量（反馈量）
	Kalman_t kalmanX = {0.001f, 0.003f, 0.03f, 0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}};
	Kalman_t kalmanY = {0.001f, 0.003f, 0.03f, 0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}};
	Attitude_t attitude = {0.0f, 0.0f, 0.0f};
	float ax, ay, az, gx, gy, gz;//原始数据
	float yaw_last;
	float gz_last;
	int roll_int, pitch_int, yaw_int;//解算数据
	
	//pid变量
	extern PID_controller  yaw_v_pid;
	extern PID_controller  yaw_pid;
											/* 切线速度pwm  YAW角速度*/ /*yaw角度*/		
	float Mechanical_zero[3]={ 0       ,   0,         0};
	float PID_yaw_v[3]= {  0,       0  ,     0 };//P、I、D 偏航角速度环，转向环内环
	float PID_yaw[3]= {  800,       0  ,     0 };//P、I、D 偏航角度环，转向环外环。前进：p--900
	int count1, count2;
	int duty;//输出量
	
	//蓝牙控制运动模块
	int in1_left, in1_right, in2_left, in2_right;//双轮pwm控制量
	uint8_t zhaunxiang_stop_flag = 0;//转向环标志位
	uint8_t rx_buffer;//接收字节
	uint8_t state = 0;//蓝牙状态机状态
	
	//雷达模块
	extern uint8_t uart4_rx_con;       //接收计数器
	extern uint8_t uart4_rx_chksum;      //异或校验
	extern uint8_t uart4_rx_buf[100];     //接收缓冲
	extern uint8_t uart4_tx_buf[10];     //接收缓冲
	extern uint8_t uart4_rx_data;  // ???????
	extern LaserPointTypeDef ax_ls_point[250];
	uint8_t time_str[27];
	
	//
	uint8_t left_flag = 1;//
	uint8_t right_flag = 1;

	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//编码器模块
//确保10ms读取一次
void Update_encoder(void)
{
	if(uwTick - systick  < 10)
		return;
	systick = uwTick;
	
	encoder_A_n = -Read_Encoder(2);//读取编码器计数值 
	encoder_B_n = Read_Encoder(1);
}

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	//开启编码器通道
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	

	//pwm模块，开启pwm
	 HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	 
	 //mpu6500
	 while(1)
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

	 //pid
	 //pid_init_yaw_v();
	 pid_init_yaw();
	
	 //开启接收中断
	 HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);//蓝牙串口
		
		//雷达模块
		HAL_Delay(500); // 等待雷达上电初始化
	 AX_LASER_Start();

	 //所有模块准备就绪，开启定时器
	 HAL_TIM_Base_Start_IT(&htim4);
	 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//更新编码器数据
		Update_encoder();//每隔10ms读取一次
		
		/*
		//发送imu数据
		char buff[50] ;
		sprintf(buff, "Yaw: %d.%02d, gz:%.2f, duty:%d\r\n",
               yaw_int / 100, abs(yaw_int % 100),gz, duty);//格式化输出字符串
		//sprintf(buff, "Roll");//格式化输出字符串
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
		*/
		
		//蓝牙控制状态机
		switch(state)
		{
			case 0: //停止
			{
				stop();
				
				char buff[50] ;
				sprintf(buff, "Roll: %d.%02d, Pitch: %d.%02d, Yaw: %d.%02d, gz:%f\r\n",
               roll_int / 100, abs(roll_int % 100),
               pitch_int / 100, abs(pitch_int % 100),
               yaw_int / 100, abs(yaw_int % 100), gz);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
				HAL_Delay(500); 
				//HAL_UART_Transmit_IT(&huart1, "0", 1);
				break;
			}
			case 1: //前进
			{
				forward();
				
				char buff[50] ;
				sprintf(buff, "Roll: %d.%02d, Pitch: %d.%02d, Yaw: %d.%02d, duty:%d, gz:%d\r\n",
               roll_int / 100, abs(roll_int % 100),
               pitch_int / 100, abs(pitch_int % 100),
               yaw_int / 100, abs(yaw_int % 100), duty, gz);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
				HAL_Delay(500); 
				/*
				char buff[50] ;
				sprintf(buff, "EncoderA: %d, EncoderB: %d\r\n", encoder_A_n, encoder_B_n);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));//通过UART1发送数据
				HAL_Delay(500); 
				//HAL_UART_Transmit_IT(&huart1, "1", 1);
				*/
				break;
			}
			case 2: //后退
			{
				back();
				
				char buff[50] ;
				sprintf(buff, "EncoderA: %d, EncoderB: %d\r\n", encoder_A_n, encoder_B_n);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));//通过UART1发送数据
				HAL_Delay(500); 
				//HAL_UART_Transmit_IT(&huart1, "2", 1);
				break;
			}
			case 3: //左转
			{
				left();
				
				char buff[50] ;
				sprintf(buff, "Roll: %d.%02d, Pitch: %d.%02d, Yaw: %d.%02d, duty:%d, gz:%f\r\n",
               roll_int / 100, abs(roll_int % 100),
               pitch_int / 100, abs(pitch_int % 100),
               yaw_int / 100, abs(yaw_int % 100), duty, gz);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
				HAL_Delay(500); 
				/*
				char buff[50] ;
				sprintf(buff, "EncoderA: %d, EncoderB: %d\r\n", encoder_A_n, encoder_B_n);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));//通过UART1发送数据
				HAL_Delay(500); 
				*/
				//HAL_UART_Transmit_IT(&huart1, "3", 1);
				break;
			}
			case 4: //右转
			{
				right();
				
				char buff[50] ;
				sprintf(buff, "EncoderA: %d, EncoderB: %d\r\n", encoder_A_n, encoder_B_n);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));//通过UART1发送数据
				HAL_Delay(500); 
				//HAL_UART_Transmit_IT(&huart1, "4", 1);
				break;
			}
			case 5: //发送雷达数据
			{
				for(int i = 0; i < 250; i+=10)
				{
					sprintf(time_str, "A:%d.%d; D:%d mm\r\n", ax_ls_point[i].angle/100, ax_ls_point[i].angle%100, ax_ls_point[i].distance); 
					HAL_UART_Transmit_IT(&huart1, time_str, sizeof(time_str));
					HAL_Delay(500); 
				}
				break;
			}
			case 6: //发送imu数据
			{
				char buff[50] ;
				sprintf(buff, "ax:%.2f,ay:%.2f,az:%.2f, gx:%.2f,gy:%.2f,gz:%.2f\r\n",
               ax,ay,az,gx,gy,gz);
				/*sprintf(buff, "Roll: %d.%02d, Pitch: %d.%02d, Yaw: %d.%02d\r\n",
               roll_int / 100, abs(roll_int % 100),
               pitch_int / 100, abs(pitch_int % 100),
               yaw_int / 100, abs(yaw_int % 100));//格式化输出字符串*/
				//sprintf(buff, "Yaw: %d.%02d, gz:%.2f\r\n",
				//					 yaw_int / 100, abs(yaw_int % 100),gz);//格式化输出字符串
				//sprintf(buff, "Roll");//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));
				HAL_Delay(500); 
				break;
			}
			case 7: //发送编码器数据
			{
				char buff[50] ;
				sprintf(buff, "EncoderA: %d, EncoderB: %d\r\n", encoder_A_n, encoder_B_n);//格式化输出字符串
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)buff, strlen(buff));//通过UART1发送数据
				HAL_Delay(500); 
				break;
			}
			default:
			{
				break;
			}
		}
		
		
		
		HAL_Delay(100);
		
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim4) {
			 
			  count1++;//分时器
			
			  //imu数据获取
        static float dt = 0.001f;
        MPU6500_Read_Accel(&hi2c1, &ax, &ay, &az);
        MPU6500_Read_Gyro(&hi2c1, &gx, &gy, &gz);
				//对原始三轴角速度处理：处理零偏和抖动问题
				gz = gz +0.5;
				if(gz <= 1.4 && gz >= -1.4) gz = 0;
				gz = gz_last*0.3 + gz*0.7;
				gz_last = gz;

        Attitude_Update(ax, ay, az, gx, gy, gz, yaw_last, dt);

        roll_int = (int)(attitude.roll * 100); // 转换为整数 (e.g., 12.34 -> 1234)
        pitch_int = (int)(attitude.pitch * 100);
        yaw_int = (int)(attitude.yaw * 100);
			
			
				//偏航角度转向环
				if(count1 == 10)
				{
					count1=0;
					
					//duty=(int)(increment__yaw(&yaw_pid, attitude.yaw));//目标量是俯仰角度，反馈量是俯仰角度，输出量是电机占空比
					
					///Set_PWM_right(in1_right,in2_right);//右轮反转
					//Set_PWM_left(in1_left,in2_left);//左轮正转
					
					if(state == 0)//停止
					{
						Set_PWM_right(0,0);//右轮反转
						Set_PWM_left(0,0);//左轮正转			
					}
					if(state == 2)//后退
					{
						Set_PWM_right(in1_right,in2_right);//右轮反转
						Set_PWM_left(in1_left,in2_left);//左轮正转			
					}
					if(state == 1)//前进
					{
						duty=(int)(increment__yaw(&yaw_pid, attitude.yaw));//目标量是俯仰角度，反馈量是俯仰角度，输出量是电机占空比
						Set_PWM_right(in1_right+duty,in2_right);//右轮反转
						Set_PWM_left(in1_left,in2_left-duty);//左轮正转			
					}
					if(state == 3)//左转
					{
						if(left_flag)		
						{
							Change_yaw_setpoint(&yaw_pid,yaw_pid.setpoint+90);
							left_flag = 0;
						}
						duty=(int)(increment__yaw(&yaw_pid, attitude.yaw));//目标量是俯仰角度，反馈量是俯仰角度，输出量是电机占空比
						if(duty > 0)
						{
							Set_PWM_right(duty,0);//右轮反转
							Set_PWM_left(duty,0);//左轮正转		
						}
						else
						{
							Set_PWM_right(0,0);//右轮反转
							Set_PWM_left(0,0);//左轮正转		
						}
							
						
						}
					if(state == 4)//右转
					{
						if(right_flag)		
						{
							Change_yaw_setpoint(&yaw_pid,yaw_pid.setpoint-90);
							right_flag = 0;
						}
						duty=(int)(increment__yaw(&yaw_pid, attitude.yaw));//目标量是俯仰角度，反馈量是俯仰角度，输出量是电机占空比
						if(duty < 0)
						{
							Set_PWM_right(0,-duty);//右轮反转
							Set_PWM_left(0,-duty);//左轮正转		
						}
						else
						{
							Set_PWM_right(0,0);//右轮反转
							Set_PWM_left(0,0);//左轮正转		
						}
					}
					
					
				}
				
				
				/*
				if(count1 == 10 && zhaunxiang_stop_flag == 1)//不加转向环
				{
					count1=0;
					
					//控制电机前进
					//Set_PWM_right(4200,0);
					Set_PWM_right(pwm_right,turn_right);
					Set_PWM_left(turn_left,pwm_left);
					*/
				}
			
			}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  
	//蓝牙控制模块  
	if (huart->Instance == USART1) 
	{
			if(rx_buffer == '0')//停止
				state = 0;
			if(rx_buffer == '1')//前进
				state = 1;
			if(rx_buffer == '2')//后退
				state = 2;
			if(rx_buffer == '3')//左转
			{state = 3;
			left_flag = 1;}
			if(rx_buffer == '4')//右转
				state = 4;
			if(rx_buffer == '5')//发送雷达数据
				state = 5;
			if(rx_buffer == '6')//发送imu数据
				state = 6;
			if(rx_buffer == '7')//发送编码器数据
				state = 7;
			
			HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
    }
	
	if (huart->Instance == USART6)
    {
        uint8_t Res = uart4_rx_data;
			
        uint8_t temp;
			
        if (uart4_rx_con < 3)
        {
            if(uart4_rx_con == 0)  //????1 
            {
                //????1
                if((Res>>4) == LS_HEADER1)
                {
                    uart4_rx_buf[uart4_rx_con] = Res;
                    uart4_rx_con = 1;                    
                }
            }
            else if(uart4_rx_con == 1) //????2 
            {
                //????2
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
            else  //???????
            {
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_con = 3;
                
                //????
                uart4_rx_chksum = Res;	
            }
        }			
        else  //????
        {
            //???????
            if(uart4_rx_con < (LS_F_LEN-1))
            {
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_con++;
                uart4_rx_chksum = uart4_rx_chksum^Res;
            }
            else
            {
                //????????
                uart4_rx_buf[uart4_rx_con] = Res;
                uart4_rx_chksum = uart4_rx_chksum^Res;
                
                //??
                uart4_rx_con = 0;
                
                //?????????
                temp = ((uint8_t)(uart4_rx_buf[1]<<4)) + (uint8_t)(uart4_rx_buf[0]&0x0F);
                
                //????????
                if( uart4_rx_chksum == temp)
                {
                    //????,???????
                    LS_DataHandle();	
                }
            }
        }
        
        // ?????????
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
