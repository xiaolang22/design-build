#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "my_systick.h"
#include <math.h>
#include "string.h"

#define WINDOW_SIZE 5	// 滤波器窗口大小
#define WINDOW_SIZE_2 5

typedef struct {
    double window[WINDOW_SIZE];
    int index;
    int count;
} MovingWindow;

//滑动窗口中值+均值混合滤波函数
double MovingWindowFilter_Update(MovingWindow* mw, double new_value);

//位置
double ReadDistance_A(void);
double ReadDistance_B(void);
int64_t ReadLines_A(void);
int64_t ReadLines_B(void);

//速度
double GetSpeed_A(void);
double GetSpeed_A_filter(void);
double GetSpeed_B(void);
double GetSpeed_B_filter(void);

#endif
