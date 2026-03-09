#include "mpu6500.h"
#include "main.h"
#include "spi.h"
#include "math.h"

// 定义CS引脚操作宏
#define MPU6500_CS_LOW()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define MPU6500_CS_HIGH()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)

Attitude_t attitude = {0.0f, 0.0f, 0.0f, 0.0f};	//解析出来的数据

////////////////////////需要用到////////////////////////
// 初始化MPU6500
void MPU6500_Init(void) {
    HAL_Delay(100); // 上电稳定延时
    // 复位设备
    MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x80);
    HAL_Delay(100); // 等待复位完成
    // 唤醒设备
    MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x00);
//    // 检查设备ID - 可选但推荐
//    uint8_t id = MPU6500_Read_Byte(MPU6500_WHO_AM_I_REG);
//    if(id != 0x70) { // MPU6500的ID是0x70
//        // 设备ID错误，可以在这里处理错误
//        // 例如: Error_Handler();
//    }
    // 配置电源管理，选择最佳时钟源
    MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x01); // 使用PLL作为时钟源
    // 设置采样率分频器
    MPU6500_Write_Byte(MPU6500_SMPLRT_DIV_REG, 0x07); // 1000/(1+7) = 125Hz
    // 配置数字低通滤波器
    MPU6500_Write_Byte(MPU6500_CONFIG_REG, 0x06); // 5Hz带宽
    // 配置陀螺仪量程为±250dps
    MPU6500_Write_Byte(MPU6500_GYRO_CONFIG_REG, 0x00);
    // 配置加速度计量程为±2g
    MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG_REG, 0x00);
    // 配置中 - 可选
    // MPU6500_Write_Byte(MPU6500_INT_ENABLE_REG, 0x00); // 禁用所有中断
    HAL_Delay(50); // 给设备一些时间完成配置
}

////////////////////////需要用到（间接）////////////////////////
// 写入MPU6500寄存器
void MPU6500_Write_Byte(uint8_t reg, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg & 0x7F;  // 写操作，最高位清零
    tx_buffer[1] = data;
    
    MPU6500_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx_buffer, 2, HAL_MAX_DELAY);
    MPU6500_CS_HIGH();
    
    // 添加微小延时以确保操作完成
    HAL_Delay(1);
}

// 读取MPU6500寄存器
uint8_t MPU6500_Read_Byte(uint8_t reg) {
    uint8_t tx_data = reg | 0x80;  // 读取操作，最高位置1
    uint8_t rx_data = 0;
    
    MPU6500_CS_LOW();
    
    // 发送寄存器地址
    HAL_SPI_Transmit(&hspi2, &tx_data, 1, HAL_MAX_DELAY);
    
    // 接收数据
    HAL_SPI_Receive(&hspi2, &rx_data, 1, HAL_MAX_DELAY);
    
    MPU6500_CS_HIGH();
    
    return rx_data;
}


////////////////////////需要用到（间接）////////////////////////
// 读取MPU6500数据 - 彻底重写
void MPU6500_Read_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz ,int16_t *temp) {
    uint8_t tx_data[15];  // 1个地址字节 + 14个虚拟字节
    uint8_t rx_data[15];  // 同上
    
    // 清空发送缓冲区
    for(int i = 0; i < 15; i++) {
        tx_data[i] = 0xFF;
        rx_data[i] = 0;
    }
    
    // 设置读取操作和起始寄存器地址
    tx_data[0] = MPU6500_ACCEL_XOUT_H_REG | 0x80;
    
    MPU6500_CS_LOW();
    
    // 使用HAL_SPI_TransmitReceive进行SPI全双工传输
    HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 15, HAL_MAX_DELAY);
    
    MPU6500_CS_HIGH();
    
    // 有效数据从rx_data[1]开始，因为第一个字节是发送地址时收到的无效数据
    *ax = ((int16_t)rx_data[1] << 8) | rx_data[2];
    *ay = ((int16_t)rx_data[3] << 8) | rx_data[4];
    *az = ((int16_t)rx_data[5] << 8) | rx_data[6];
    // 温度数据在rx_data[7]和rx_data[8]，这里跳过
    *temp = ((int16_t)rx_data[7] << 8) | rx_data[8];   
    *gx = ((int16_t)rx_data[9] << 8) | rx_data[10];
    *gy = ((int16_t)rx_data[11] << 8) | rx_data[12];
    *gz = ((int16_t)rx_data[13] << 8) | rx_data[14];
}

////////////////////////需要用到////////////////////////
// 读取MPU6500数据（浮点数），自己写的
void MPU6500_Read_Data_float(float *ax_f, float *ay_f, float *az_f, float *gx_f, float *gy_f, float *gz_f, float *temp)
{
	//读取原始数据（int16_t）
	int16_t ax, ay, az, gx, gy, gz, temp_raw;
	MPU6500_Read_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp_raw);
	
	//转换为浮点数
	*ax_f = ax / 16384.0f; // ±8g  加速度（g）
	*ay_f = ay / 16384.0f;
	*az_f = az / 16384.0f;
	
	*gx_f = gx / 131.0f; // ±2000°/s  角速度（°/s）
	*gy_f = gy / 131.0f;
	*gz_f = gz / 131.0f;
	
	*temp = (float)temp_raw / 340.0f + 36.53f;  // 温度（摄氏度）
}

//更新Yaw角
//gz:这一次测得的角速度(假设dt时间内角速度不变)，yaw_last:上一次计算出来的yaw，dt:上一次计算yaw和这一次计算yaw的时间差
//yaw单位是度
void Yaw_Update(float gz, float yaw_last, float dt)
{
		attitude.yaw += gz * dt;	//计算yaw角增量并累加到上一次计算值，假设dt时间内角速度不变
    if (attitude.yaw > 360.0f) attitude.yaw -= 360.0f;
    if (attitude.yaw < 0.0f) attitude.yaw += 360.0f;
		if(fabs(attitude.yaw - yaw_last) < 0.01) attitude.yaw = yaw_last;	//限幅滤波，消除累积误差
		
		if(attitude.yaw>180.0f && attitude.yaw<360.0f) attitude.yaw = attitude.yaw - 360.0f; //修正到-180--180
	
		yaw_last = attitude.yaw;//更新yaw_last
}

