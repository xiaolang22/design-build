#include "mpu6500.h"

//extern Kalman_t kalmanX;
//extern Kalman_t kalmanY;
//extern Attitude_t attitude;

Kalman_t kalmanX = {0.001f, 0.003f, 0.03f, 0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}};
Kalman_t kalmanY = {0.001f, 0.003f, 0.03f, 0.0f, 0.0f, {{0.0f, 0.0f}, {0.0f, 0.0f}}};
Attitude_t attitude = {0.0f, 0.0f, 0.0f};

/*
void MPU6500_Init(void) {
    uint8_t data;
    HAL_Delay(100); // 等待设备稳定

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, MPU6500_WHO_AM_I, 1, &data, 1, 100) != HAL_OK) {
        Error_Handler();
    }
    if (data != 0x70) {
        Error_Handler();
    }

    data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, MPU6500_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK) {
        Error_Handler();
    }
    data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, MPU6500_GYRO_CONFIG, 1, &data, 1, 100) != HAL_OK) {
        Error_Handler();
    }
    data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, MPU6500_ACCEL_CONFIG, 1, &data, 1, 100) != HAL_OK) {
        Error_Handler();
    }
}
*/

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    HAL_Delay(100); // 等待设备稳定

    if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, MPU6500_WHO_AM_I, 1, &data, 1, 100) != HAL_OK) {
				return 0;
    }
    if (data != 0x70) {
        return 0;
    }

    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, MPU6500_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK) {
        return 0;
    }
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, MPU6500_GYRO_CONFIG, 1, &data, 1, 100) != HAL_OK) {
        return 0;
    }
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, MPU6500_ACCEL_CONFIG, 1, &data, 1, 100) != HAL_OK) {
        return 0;
    }
		
		return 1;
}

void MPU6500_Read_Accel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az) {
    uint8_t buffer[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, MPU6500_ACCEL_XOUT_H, 1, buffer, 6, 100) == HAL_OK) {
        int16_t raw_ax = (buffer[0] << 8) | buffer[1];
        int16_t raw_ay = (buffer[2] << 8) | buffer[3];
        int16_t raw_az = (buffer[4] << 8) | buffer[5];
        *ax = raw_ax / 16384.0f; // ±8g
        *ay = raw_ay / 16384.0f;
        *az = raw_az / 16384.0f;
    } else {
        *ax = *ay = *az = 0.0f; // 错误时返回 0
				// I2C异常，重新初始化
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(100);
        MX_I2C1_Init();
        MPU6500_Init(&hi2c1);
    }
}

void MPU6500_Read_Gyro(I2C_HandleTypeDef *hi2c, float *gx, float *gy, float *gz) {
    uint8_t buffer[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, MPU6500_GYRO_XOUT_H, 1, buffer, 6, 100) == HAL_OK) {
        int16_t raw_gx = (buffer[0] << 8) | buffer[1];
        int16_t raw_gy = (buffer[2] << 8) | buffer[3];
        int16_t raw_gz = (buffer[4] << 8) | buffer[5];
        *gx = raw_gx / 131.0f; // ±2000°/s
        *gy = raw_gy / 131.0f;
        *gz = raw_gz / 131.0f;
    } else {
        *gx = *gy = *gz = 0.0f; // 错误时返回 0
				// I2C异常，重新初始化
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(100);
        MX_I2C1_Init();
        MPU6500_Init(&hi2c1);
    }
}

float Kalman_Filter(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    float rate = newRate - kalman->bias;
    kalman->angle += dt * rate;

    kalman->P[0][0] += dt * (kalman->P[1][1] * kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] = kalman->P[0][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    float K[2];
    float S = kalman->P[0][0] + kalman->R_measure;
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    kalman->angle += K[0] * (newAngle - kalman->angle);
    kalman->bias += K[1] * (newAngle - kalman->angle);

    kalman->P[0][0] -= K[0] * kalman->P[0][0];
    kalman->P[0][1] -= K[0] * kalman->P[0][1];
    kalman->P[1][0] -= K[1] * kalman->P[0][0];
    kalman->P[1][1] -= K[1] * kalman->P[0][1];

    return kalman->angle;
}

void Attitude_Update(float ax, float ay, float az, float gx, float gy, float gz, float yaw_last, float dt) {
    float acc_roll = atan2f(ay, az) * 180.0f / My_PI;
    float acc_pitch = -atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / My_PI;

    attitude.roll = Kalman_Filter(&kalmanX, acc_roll, gx, dt);
    attitude.pitch = Kalman_Filter(&kalmanY, acc_pitch, gy, dt);

//    attitude.yaw += gz * dt;
//    if (attitude.yaw > 360.0f) attitude.yaw -= 360.0f;
//    if (attitude.yaw < 0.0f) attitude.yaw += 360.0f;
//		if(fabs(attitude.yaw - yaw_last) < 0.01) attitude.yaw = yaw_last;
//		
//		if(attitude.yaw>180.0f && attitude.yaw<360.0f) attitude.yaw = attitude.yaw - 360.0f; //修正到-180--180
//	
//		yaw_last = attitude.yaw;
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




