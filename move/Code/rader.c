#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "rader.h"

uint8_t uart4_rx_con=0;       //���ռ�����
uint8_t uart4_rx_chksum;      //���У��
uint8_t uart4_rx_buf[100];     //���ջ���
uint8_t uart4_tx_buf[10];     //���ջ���


uint8_t uart4_rx_data = 0;  // ???????

//ɨ��һȦ���״�����

LaserPointTypeDef ax_ls_point[250];

/**
  * @��  ��  ���ݴ�����
  * @��  ��  ��
  * @����ֵ	 ��
  */
void LS_DataHandle(void)
{

	uint8_t i;
	float temp;
	
	static uint16_t cnt = 0;
	
	static float angle_last = 0;
	
	//ÿ��ɼ�5000��10HZ��תһȦ�ɼ�500���㣬���㵥Ƭ������2����ȡһ����
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

    

	
	//��ʼ�Ƕȴ��ڽ����Ƕȣ����360��
	if(angle_new > angle_last)
	{
		angle_area  = (angle_new - angle_last)/20;
		
		for(i=0; i<20; i++)
		{
			temp = angle_new + angle_area*i;
			
			//����Ƕ�
			if(temp > 360)
			{
				point[cnt+i].angle = (temp - 360) * 100;

			}
			else
			{
				point[cnt+i].angle = (temp) * 100;

			}
			
			//�������
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
			
			//�������
			point[cnt+i].distance =  ((uint16_t)(uart4_rx_buf[5+i*4]<<8)) + (uint8_t)uart4_rx_buf[4+i*4];
		}
		
	}

	//��ֵ��һ�β����Ƕ�
	angle_last = angle_new;	
	
	//�����������
	//printf("%d %d %d \r\n",cnt, point[0].angle, point[0].distance);	

	//һ֡���ݽ�������
	cnt = cnt+20;
	
	//�ж��Ƿ�ת��һȦ���״�תһȦ��250���㣩
	  
	if(cnt > 260)
	{
		//�����������ת�Ƶ��ⲿ�����У����⸲������
		for(i=0; i<250; i++)
		{
			//����Ƕ�
			ax_ls_point[i].angle = point[i].angle;
			ax_ls_point[i].distance = point[i].distance;
		}
		
		//��λ
		cnt = 0;
	}

}


/**
  * @��  ��  �״���������ʵģʽ��
  * @��  ��  ��
  * @����ֵ	 ��
  */
void AX_LASER_Start(void)   
{
	uint8_t i;	
	
	uart4_tx_buf[0] = 0xA5;  //֡ͷ
	uart4_tx_buf[1] = 0x82;  //����ɨ������
	uart4_tx_buf[2] = 05;    
	uart4_tx_buf[3] = 0;    
	uart4_tx_buf[4] = 0;    
	uart4_tx_buf[5] = 0;    
	uart4_tx_buf[6] = 0;    
	uart4_tx_buf[7] = 0;    
	uart4_tx_buf[8] = 0x22;  //У���
	

	if (HAL_UART_Receive_IT(&huart6, &uart4_rx_data, 1) != HAL_OK) {
      Error_Handler();
  }
  	HAL_Delay(2000);
	
	//��ѯ���䷽ʽ
	for(i=0; i<9; i++)
	{
		HAL_UART_Transmit(&huart6, &uart4_tx_buf[i], 1, 100);
		while (HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
		
		HAL_UART_Transmit(&huart1,"one bit sent", 12, 100);

	}
}


/**
  * @��  ��  �״�ر�
  * @��  ��  ��
  * @����ֵ	 ��
  */
void AX_LASER_Stop(void)   
{
	uint8_t i;	

	uart4_tx_buf[0] = 0xA5;       //֡ͷ
	uart4_tx_buf[1] = 0x25;       //�ر�����
	uart4_tx_buf[2] = 0xA5+0x25;  //У���
	
	//��ѯ���䷽ʽ
	for(i=0; i<3; i++)
	{
//		USART_SendData(UART4, uart4_tx_buf[i]);
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TC) != SET);
		
		
		HAL_UART_Transmit(&huart6, &uart4_tx_buf[i], 1, 100);
		while (HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);
	}
}

