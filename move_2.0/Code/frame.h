/*
	传感器数据帧通信模块
	本文件用于imu、编码器、雷达数据帧的封装、上传
	
	完成数据帧格式定义
	完成数据的字节序转换，在RAM中数据以大端字节序(低地址存高位数据)存储，方便适应DMA从小地址开始读取数据
*/

#ifndef __FRAME_H
#define __FRAME_H

#include "main.h"

//***********************完成与上位机通信***********************//
// 数据帧结构定义
#define FRAME_HEADER 0xFF77
#define FRAME_TAIL 0x77EE
#define IMU_RAW_DATA_LENGTH 6
#define IMU_RPY_DATA_LENGTH 3
#define LASER_DATA_POINTS 250
#define LASER_ANGLE_RESOLUTION 1.44f

//	564字节/帧
#pragma pack(push, 1)					//开启内存对齐
typedef struct {
    uint16_t header;          										// 帧头 0xFF77
		float imu_raw_data[IMU_RAW_DATA_LENGTH];			// imu原始数据（imu）
		float imu_rpy_data[IMU_RPY_DATA_LENGTH];			// imu欧拉角（imu）
		double wheelA_dis;														// A轮距离（编码器）
		double wheelB_dis;														// B轮距离（编码器）
    uint16_t point_count;    											// 点数（雷达）
    float angle_resolution;   										// 角度分辨率（雷达）
    uint16_t distances[LASER_DATA_POINTS]; 				// 距离数据数组（雷达）
    uint16_t checksum;        										// 校验和
    uint16_t tail;            										// 帧尾 0x77EE
} Frame;
#pragma pack(pop)							//恢复默认对齐模式

// 字节序转换
uint16_t swap_uint16(uint16_t val);
uint32_t swap_uint32(uint32_t val);
float swap_float(float f);
double swap_double(double d);


#endif