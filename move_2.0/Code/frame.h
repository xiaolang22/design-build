/*
	����������֡ͨ��ģ��
	���ļ�����imu�����������״�����֡�ķ�װ���ϴ�
	
	�������֡��ʽ����
	������ݵ��ֽ���ת������RAM�������Դ���ֽ���(�͵�ַ���λ����)�洢��������ӦDMA��С��ַ��ʼ��ȡ����
*/

#ifndef __FRAME_H
#define __FRAME_H

#include "main.h"

//***********************�������λ��ͨ��***********************//
// ����֡�ṹ����
#define FRAME_HEADER 0xFF77
#define FRAME_TAIL 0x77EE
#define IMU_RAW_DATA_LENGTH 6
#define IMU_RPY_DATA_LENGTH 3
#define LASER_DATA_POINTS 250
#define LASER_ANGLE_RESOLUTION 1.44f

//	564�ֽ�/֡
#pragma pack(push, 1)					//�����ڴ����
typedef struct {
    uint16_t header;          										// ֡ͷ 0xFF77
		float imu_raw_data[IMU_RAW_DATA_LENGTH];			// imuԭʼ���ݣ�imu��
		float imu_rpy_data[IMU_RPY_DATA_LENGTH];			// imuŷ���ǣ�imu��
		double wheelA_dis;														// A�־��루��������
		double wheelB_dis;														// B�־��루��������
    uint16_t point_count;    											// �������״
    float angle_resolution;   										// �Ƕȷֱ��ʣ��״
    uint16_t distances[LASER_DATA_POINTS]; 				// �����������飨�״
    uint16_t checksum;        										// У���
    uint16_t tail;            										// ֡β 0x77EE
} Frame;
#pragma pack(pop)							//�ָ�Ĭ�϶���ģʽ

// �ֽ���ת��
uint16_t swap_uint16(uint16_t val);
uint32_t swap_uint32(uint32_t val);
float swap_float(float f);
double swap_double(double d);


#endif