#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"
#include <stdio.h>
#include <math.h>

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

#define MPU6500_ADDR 0x68 << 1 // I2C 地址 (AD0 接地)
#define MPU6500_WHO_AM_I 0x75
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_GYRO_XOUT_H 0x43
#define My_PI 3.14159265358979323846  // 手动定义兜底

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c);
void MPU6500_Read_Accel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az);
void MPU6500_Read_Gyro(I2C_HandleTypeDef *hi2c, float *gx, float *gy, float *gz);
float Kalman_Filter(Kalman_t *kalman, float newAngle, float newRate, float dt);
void Attitude_Update(float ax, float ay, float az, float gx, float gy, float gz, float yaw_last, float dt) ;


#endif
