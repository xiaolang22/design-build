/*
	传感器数据帧通信模块
	本文件用于imu、编码器、雷达数据帧的封装、上传
	
	数据的字节序转换实现
*/

#include "frame.h"

/**
  * @简  述  uint16_t字节序转换函数
  */
uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}
/**
  * @简  述  uint32_t字节序转换函数
  */
uint32_t swap_uint32(uint32_t val) {
    return ((val << 24) & 0xff000000) |
           ((val <<  8) & 0x00ff0000) |
           ((val >>  8) & 0x0000ff00) |
           ((val >> 24) & 0x000000ff);
}
/**
  * @简  述  浮点数字节序转换（通过联合体实现）
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
//  * @简  述  double字节序转换函数（通过联合体实现）
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
//    // 交换64位整数的字节序
//    // 方法1：直接交换高低32位，并对每个32位进行字节序交换
//    dest.parts.low = swap_uint32(src.parts.high);
//    dest.parts.high = swap_uint32(src.parts.low);
//    
//    return dest.d;
//}

/**
  * @简  述  double字节序转换函数（使用内存拷贝）
  */
double swap_double(double d) {
    uint8_t bytes[8];
    uint8_t swapped_bytes[8];
    
    // 将double拷贝到字节数组
    memcpy(bytes, &d, sizeof(double));
    
    // 交换字节顺序
    for (int i = 0; i < 8; i++) {
        swapped_bytes[i] = bytes[7 - i];
    }
    
    // 将交换后的字节数组拷贝回double
    double result;
    memcpy(&result, swapped_bytes, sizeof(double));
    return result;
}


