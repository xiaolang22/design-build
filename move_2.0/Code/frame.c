/*
	����������֡ͨ��ģ��
	���ļ�����imu�����������״�����֡�ķ�װ���ϴ�
	
	���ݵ��ֽ���ת��ʵ��
*/

#include "frame.h"

/**
  * @��  ��  uint16_t�ֽ���ת������
  */
uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}
/**
  * @��  ��  uint32_t�ֽ���ת������
  */
uint32_t swap_uint32(uint32_t val) {
    return ((val << 24) & 0xff000000) |
           ((val <<  8) & 0x00ff0000) |
           ((val >>  8) & 0x0000ff00) |
           ((val >> 24) & 0x000000ff);
}
/**
  * @��  ��  �������ֽ���ת����ͨ��������ʵ�֣�
  */
float swap_float(float f) {
    union {
        float f;
        uint32_t u;
    } src, dest;
    
    src.f = f;
    dest.u = swap_uint32(src.u);
    return dest.f;
}

///**
//  * @��  ��  double�ֽ���ת��������ͨ��������ʵ�֣�
//  */
//double swap_double(double d) {
//    union {
//        double d;
//        uint64_t u;
//        struct {
//            uint32_t low;
//            uint32_t high;
//        } parts;
//    } src, dest;
//    
//    src.d = d;
//    
//    // ����64λ�������ֽ���
//    // ����1��ֱ�ӽ����ߵ�32λ������ÿ��32λ�����ֽ��򽻻�
//    dest.parts.low = swap_uint32(src.parts.high);
//    dest.parts.high = swap_uint32(src.parts.low);
//    
//    return dest.d;
//}

/**
  * @��  ��  double�ֽ���ת��������ʹ���ڴ濽����
  */
double swap_double(double d) {
    uint8_t bytes[8];
    uint8_t swapped_bytes[8];
    
    // ��double�������ֽ�����
    memcpy(bytes, &d, sizeof(double));
    
    // �����ֽ�˳��
    for (int i = 0; i < 8; i++) {
        swapped_bytes[i] = bytes[7 - i];
    }
    
    // ����������ֽ����鿽����double
    double result;
    memcpy(&result, swapped_bytes, sizeof(double));
    return result;
}


