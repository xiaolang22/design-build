#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"

// 定义MPU6500的寄存器地址
#define MPU6500_WHO_AM_I_REG 0x75
#define MPU6500_PWR_MGMT_1_REG 0x6B
#define MPU6500_SMPLRT_DIV_REG 0x19
#define MPU6500_CONFIG_REG 0x1A
#define MPU6500_GYRO_CONFIG_REG 0x1B
#define MPU6500_ACCEL_CONFIG_REG 0x1C
#define MPU6500_ACCEL_XOUT_H_REG 0x3B
#define MPU6500_TEMP_OUT_H_REG 0x41
#define MPU6500_GYRO_XOUT_H_REG 0x43
// 定义MPU6500的设备地址
#define MPU6500_ADDRESS 0x68

// 角度结构体
typedef struct {
    float roll;
    float pitch;
    float yaw;
		float temp;
} Attitude_t;

// SPI操作函数声明
void MPU6500_Init(void);//需要 —— 初始化
void MPU6500_Write_Byte(uint8_t reg, uint8_t data);//需要（间接）
void MPU6500_Read_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz,int16_t *temp);//需要（间接）
void MPU6500_Read_Data_float(float *ax_f, float *ay_f, float *az_f, float *gx_f, float *gy_f, float *gz_f, float *temp);//需要（自定义） —— 读取浮点型原始数据
void Yaw_Update(float gz, float yaw_last, float dt);//更新偏航角（自定义）
#endif // MPU6500_H