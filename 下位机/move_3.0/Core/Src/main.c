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
	此为迷宫小车下位机代码，主要实现两个核心功能：
	1、将传感器数据封装上传
	2、接收速度控制指令并执行运动控制
		
	 目前已经实现的功能：
		【传感器部分】
		1、雷达驱动+数据读取
		2、编码器数据读取（距离（外部中断）、速度（M法））
		3、imu数据读取（spi）
		
		【执行器部分】
		1、pid控制（gz环 + 双轮速度环）
		
		【通信部分】
		1、雷达、编码器、imu数据帧上行（具体数据内容见Frame.h帧结构）
		2、速度控制指令下行
		
		更新：解决原地低速旋转容易抖动的问题(10.21)
		更新：解决原地低速旋转突然停止会抖动的问题（10.22）
		
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
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
#include "pid.h"
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

//***************************全局变量区***************************//

//*******************传感器*******************//
//编码器
double A_speed = 0, A_speed_last = 0;
double B_speed = 0,	B_speed_last = 0;										//速度计算值
double speed_linear = 0;																//线速度值
double dis_A_latest = 0, dis_A_last = 0; 								//A轮测距结果(mm)，用于速度计算
double dis_B_latest = 0, dis_B_last = 0;								//B轮测距结果(mm)，用于速度计算
uint32_t EA_time_latest = 0, EA_time_last = 0;	//A轮测距时间间隔(ms)
uint32_t EB_time_latest = 0, EB_time_last = 0;  //B轮测距时间间隔(ms)
double dis_A = 0, dis_B = 0;						//测距结果，用于里程计

//IMU模块
float ax, ay, az, gx, gy, gz, temp;			//原始数据
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

//*******************执行器*******************//
//PID控制器模块
//PID控制器
extern PID_controller A_speed_PID_controller;	//A轮速度PID控制器
extern PID_controller B_speed_PID_controller;	//B轮速度PID控制器
extern PID_controller gz_PID_controller;				//偏航角速度PID控制器
//机械零点（默认初始目标）
extern float Mechanical_zero[];//A轮速度  B轮速度  偏航角速度
//PID参数
extern float A_speed_PID_param[];		//P I D
extern float B_speed_PID_param[];		//P I D
extern float gz_PID_param[];				//P I D
//电机速度控制
int A_pwm_offset = 4000;	//pwm死区
int B_pwm_offset = 4000;
int duty_A = 0, duty_B = 0; 
double linear_speed_goal = 0;		//线速度目标值（来自上位机线速度控制指令，由上位机修改）

//*******************通信*******************//
//上行
Frame frame;	//上行数据帧
volatile uint8_t uart1_tx_dma_complete = 1;	// 串口发送模块（蓝牙）：添加DMA发送完成标志
//下行
SpeedCommandFrame speed_frame = {0};		//下行数据帧
float linear_speed_cmd = 0, angular_speed_cmd = 0;
// DMA接收相关变量
uint8_t speed_cmd_dma_buffer[SPEED_CMD_DMA_BUFFER_SIZE];  // DMA接收缓冲区
volatile uint16_t dma_received_len = 0;  // 已接收数据长度
uint16_t last_dma_counter = 0;  // 上一次DMA计数器值
extern DMA_HandleTypeDef hdma_usart1_rx;


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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
	//***************************自定义初始化***************************//
	
	//*******************传感器初始化*******************//
	//编码器模块（滴答计时器）
	HAL_TIM_Base_Start_IT(&htim2);  // 启动自定义滴答计时器
	
	//IMU模块
	MPU6500_Init();// IMU初始化
	
	//雷达模块
	HAL_Delay(500); 	// 等待雷达上电初始化
	AX_LASER_Start();	// 发送请求报文，雷达开始发送应答报文
	
	//*******************执行器初始化*******************//
	//PWM模块，开启pwm
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	//PID控制器初始化
	PID_init(&A_speed_PID_controller);
	PID_init(&B_speed_PID_controller);
	PID_init(&gz_PID_controller);
	
	//*******************通信初始化*******************//
	//上行
	//完成固定数据填充
	frame.header = swap_uint16(FRAME_HEADER);  // 转换为大端序
	frame.point_count = swap_uint16(LASER_DATA_POINTS);
	frame.angle_resolution = swap_float(LASER_ANGLE_RESOLUTION);
	frame.tail = swap_uint16(FRAME_TAIL);
	//下行
	HAL_UART_Receive_DMA(&huart1, speed_cmd_dma_buffer, SPEED_CMD_DMA_BUFFER_SIZE);	// 启动DMA接收
	last_dma_counter = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // 初始计数器值
	
	//*******************任务调度开启*******************//
	//所有模块准备就绪，开启任务调度定时器
	HAL_TIM_Base_Start_IT(&htim4);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		 if(uart1_tx_dma_complete == 1)	 //DMA空闲
//		{
//			//调试发送
//			char buff[50] ;
//			float i = 40.0;
//			sprintf(buff, "%f, %lf, %lf, %f, %lf\r\n",i, A_speed, B_speed, gz, (A_speed+B_speed)/2);
//			uart1_tx_dma_complete = 0;
//			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buff, strlen(buff));
//		}

		
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

//处理DMA接收到的速度指令数据，并更新运动控制目标
void ProcessSpeedCommandDMA(void)
{
    static uint16_t processed_index = 0;  // 已处理数据索引
    
    // 获取当前DMA计数器值
    uint16_t current_dma_counter = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    
    // 计算新接收的数据长度
    uint16_t new_data_len = (last_dma_counter >= current_dma_counter) ? 
                           (last_dma_counter - current_dma_counter) :															//一个循环内
                           (SPEED_CMD_DMA_BUFFER_SIZE - current_dma_counter + last_dma_counter);	//跨过两个循环
    
    if (new_data_len > 0) {
        // 处理新接收的数据
        for (uint16_t i = 0; i < new_data_len; i++) {
            uint16_t buffer_index = (processed_index + i) % SPEED_CMD_DMA_BUFFER_SIZE;
            uint8_t received_byte = speed_cmd_dma_buffer[buffer_index];
            
            // 速度控制指令解析状态机
            static uint8_t speed_cmd_state = 0; // 0:等待帧头1, 1:等待帧头2, 2:接收数据
            static uint8_t speed_cmd_buffer[12]; // 12字节缓冲区
            static uint8_t speed_cmd_index = 0;
            
            switch(speed_cmd_state) {
                case 0: // 等待帧头第一个字节 0xFF
                    if(received_byte == 0xFF) {
                        speed_cmd_buffer[0] = received_byte;
                        speed_cmd_index = 1;
                        speed_cmd_state = 1;
                    }
                    break;
                    
                case 1: // 等待帧头第二个字节 0x22
                    if(received_byte == 0x22) {
                        speed_cmd_buffer[1] = received_byte;
                        speed_cmd_index = 2;
                        speed_cmd_state = 2;
                    } else {
                        // 如果不是0x22，重新开始搜索帧头
                        speed_cmd_state = 0;
                        speed_cmd_index = 0;
                    }
                    break;
                    
                case 2: // 接收数据部分
                    if(speed_cmd_index < 12) {
                        speed_cmd_buffer[speed_cmd_index] = received_byte;
                        speed_cmd_index++;
                    }
                    
                    // 检查是否接收到完整帧（12字节）
                    if(speed_cmd_index >= 12) {
                        // 验证帧尾
                        if(speed_cmd_buffer[10] == 0x22 && speed_cmd_buffer[11] == 0xEE) {
                            // 解析速度指令 
                            SpeedCommandFrame cmd_frame;
                            memcpy(&cmd_frame, speed_cmd_buffer, sizeof(SpeedCommandFrame));
                            angular_speed_cmd = cmd_frame.angular_speed;
                            linear_speed_cmd = cmd_frame.linear_speed;
                        
                            // 更新PID控制器目标
														// 上位机DWA规划器配置限制了线速度指令大于0.05m/s，gz无限制
                            Change_PID_setpoint(&gz_PID_controller, angular_speed_cmd);
                            linear_speed_goal = linear_speed_cmd;
                            
//                            // 调试输出（可选）
//                             char debug_msg[50];
//                             sprintf(debug_msg, "DMA Recv: L=%.3f, A=%.3f\r\n", linear_speed_cmd, angular_speed_cmd);
//                             HAL_UART_Transmit_IT(&huart1, (uint8_t*)debug_msg, strlen(debug_msg));
                        }
                        
                        // 重置状态机，准备接收下一帧
                        speed_cmd_state = 0;
                        speed_cmd_index = 0;
                    }
                    break;
            }
        }
        
        // 更新已处理索引
        processed_index = (processed_index + new_data_len) % SPEED_CMD_DMA_BUFFER_SIZE;
    }
    
    // 更新DMA计数器值
    last_dma_counter = current_dma_counter;
}

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
int count1 = 0;//速度读取分时器
int count2 = 0;//偏航角速度控制分时器
int count3 = 0;//速度控制分时器
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim == &htim4) // 任务调度计时器
		{
			count1++;//速度读取分时器
			count2++;//偏航角速度控制分时器
			count3++;//速度控制分时器
			
			////////////////通信(1)：DMA串口接收速度控制指令，并更新运动控制目标（下行）///////////////////////////
      ProcessSpeedCommandDMA();// 新增：处理DMA接收的速度指令数据
			
			//////////////////////////传感器数据获取(1)：编码器距离//////////////////////////////////////////
			//编码器距离数据获取
			double dis_A = ReadDistance_A();
			double dis_B = ReadDistance_B();
			
			//////////////////////////传感器数据获取(2)：编码器速度//////////////////////////////////////////
			if(count1 >= 30)	//计算速度（读取频率：50hz，30ms读取一次）
			{
				count1 = 0;	//清空分时器
				
				//更新上一次
				//A
				EA_time_last = EA_time_latest;//上一次时间
				dis_A_last = dis_A_latest;//上一次距离
				//B
				EB_time_last = EB_time_latest;//上一次时间
				dis_B_last = dis_B_latest;//上一次距离
				
				//读取当前数据
				dis_B_latest = ReadDistance_B();//当前距离B
				EB_time_latest = HAL_GetTick();//记录测量时间B
				dis_A_latest = ReadDistance_A();//当前距离A
				EA_time_latest = HAL_GetTick();//记录测量时间A
				
				//计算速度
				A_speed = ((dis_A_latest - dis_A_last) / 1000.0f) / ((EA_time_latest - EA_time_last) / 1000.0f);  // m/s
				B_speed = ((dis_B_latest - dis_B_last) / 1000.0f) / ((EB_time_latest - EB_time_last) / 1000.0f);  // m/s
				
				//线速度
				speed_linear = (A_speed + B_speed) / 2;	// m/s
			}
			
			/////////////////////////传感器数据获取(3)：imu//////////////////////////////////////////
			//imu数据获取(占用DMA和总线资源)
			//读取原始数据
			MPU6500_Read_Data_float(&ax, &ay, &az, &gx, &gy, &gz, &temp);
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
			
			static uint8_t speed_pid_enable = 1;	//是否允许速度pid控制
			static double gz_setpoint_last = 0;	//记录上一次的gz目标值
			static int stable_cnt = 0;
			//////////////////////////运动控制(1)：偏航角速度控制//////////////////////////////////////////
			if(count2 >= 10)	//偏航角速度控制
			{
				count2 = 0;	//清空分时器
				
				// 特殊情况处理：线速度为0，原地旋转的时候，为了避免gz过低引起震荡，使用统一轮速控制(原地低速旋转 开环)
				if(linear_speed_goal == 0 && (gz_PID_controller.setpoint > 0 && gz_PID_controller.setpoint < 80))	//逆时针转
				{
					speed_pid_enable = 0;	//关闭速度环
					
					Set_PWM_A(4500, 0);	//A轮正转，B轮反转
					Set_PWM_B(4500, 0);
					
					gz_setpoint_last = gz_PID_controller.setpoint;
				}
				else if(linear_speed_goal == 0 && (gz_PID_controller.setpoint < 0 && gz_PID_controller.setpoint > -80))	//顺时针转
				{
					speed_pid_enable = 0;	//关闭速度环
					
					Set_PWM_A(0, 4500);	//A轮反转，B轮正转
					Set_PWM_B(0, 4500);
					
					gz_setpoint_last = gz_PID_controller.setpoint;
				}
				else if(linear_speed_goal == 0 && (gz_PID_controller.setpoint == 0 && gz_setpoint_last != 0))		//从原地旋转切换到停止
				{
					speed_pid_enable = 0;
					
					Set_PWM_A(0, 0);
					Set_PWM_B(0, 0);
					
					gz_setpoint_last = gz_PID_controller.setpoint;
				}
				else	// 回归一般情况
				{
					if(gz_setpoint_last == 0)	//刚从原地旋转切换到停止，稳定一下
					{
						stable_cnt++;
						
						if(stable_cnt > 5)
						{
							stable_cnt = 0;
							gz_setpoint_last = -1;
						}
					}
					else
					{
						speed_pid_enable = 1;	//开启速度环
					
						double delta = PID_increment(&gz_PID_controller, gz);//计算速度偏差量
					
						Change_PID_setpoint(&A_speed_PID_controller, linear_speed_goal - delta);
						Change_PID_setpoint(&B_speed_PID_controller, linear_speed_goal + delta);
					}
					
				}
			}
			//////////////////////////运动控制(2)：A、B电机速度控制//////////////////////////////////////////
			if(count3 >= 20 && speed_pid_enable)	//A、B轮速度控制
			{
				count3 = 0;	//清空分时器
				
				//正转
				if(A_speed_PID_controller.setpoint > 0.03)
				{
					duty_A = (int)PID_increment(&A_speed_PID_controller, A_speed);//A轮速度控制计算
					Set_PWM_A(A_pwm_offset + duty_A, 0);//执行速度控制
				}
				if(B_speed_PID_controller.setpoint > 0.03)
				{
					duty_B = (int)PID_increment(&B_speed_PID_controller, B_speed);//B轮速度控制计算
					Set_PWM_B(0, B_pwm_offset + duty_B);//执行速度控制
				}
				
				//反转
				if(A_speed_PID_controller.setpoint < -0.03)
				{
					duty_A = (int)PID_increment(&A_speed_PID_controller, A_speed);//A轮速度控制计算
					Set_PWM_A(0,A_pwm_offset - duty_A);//执行速度控制
				}
				if(B_speed_PID_controller.setpoint < -0.03)
				{
					duty_B = (int)PID_increment(&B_speed_PID_controller, B_speed);//B轮速度控制计算
					Set_PWM_B(B_pwm_offset - duty_B, 0);//执行速度控制
				}
				//停止
				if(fabs(A_speed_PID_controller.setpoint) <= 0.03)
				{
					Set_PWM_A(0,0);
				}
				if(fabs(B_speed_PID_controller.setpoint) <= 0.03)
				{
					Set_PWM_B(0,0);
				}
			}
			
			////////////////通信(2)：DMA串口发送imu、编码器、雷达数据帧到上位机（上行）///////////////////////////
      if(uart1_tx_dma_complete == 1)	 //DMA空闲
			{
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
				
				//装载线速度数据帧
				frame.speed_linear = swap_double(speed_linear);	//转换为大端字节序
				calc_checksum += (uint16_t)(frame.speed_linear * 100);	//计算校验和
				
				//装载雷达数据帧
				LaserPointTypeDef temp_points[250];	// 复制雷达数据到临时数组，避免数据竞争	
				__disable_irq();// 短暂禁用中断确保数据一致性
				memcpy(temp_points, ax_ls_point, sizeof(ax_ls_point));
				__enable_irq();
				
				for(int i = 0; i < LASER_DATA_POINTS - 1; i++) // 雷达数据按角度从大到小排序（适应ROS标准：默认雷达逆时针扫描）
				{
						for(int j = 0; j < LASER_DATA_POINTS - i - 1; j++) 
						{
							if(temp_points[j].angle < temp_points[j+1].angle) 
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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	/////////////////////////传感器数据获取(4)：雷达//////////////////////////////////////////
	// 雷达数据接收区(8ms发一帧数据)
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
				//开启下一次接收中断
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
