#include "encoder.h"

// 编码器参数
#define LINES_PER_TURN 780 					 // 输出轴转一圈对应编码器线数(13对，26线，减速比30)
#define DISTANCE_PER_LINE_mm 213/780 // 每根线对应前进距离（约为0.273mm每线）

// 测位置变量
int64_t encoderA_line = 0;	//A编码器线数
int64_t encoderB_line = 0;	//B编码器线数
double wheelA_dis = 0.0f;		//A轮前进距离
double wheelB_dis = 0.0f;		//B轮前进距离

// 测速变量
int64_t t0_A, t1_A, t0_B, t1_B;	//前后两个边沿的时刻
int direction_A, direction_B;	//A、B轮的转向（+1代表正转，-1代表反转，+2代表由反转变成正转，-2代表由正转变成反转）

// 滤波器变量
MovingWindow mw_A = {{0}, 0, 0};
MovingWindow mw_B = {{0}, 0, 0};
MovingWindow mw_A_1 = {{0}, 0, 0};
MovingWindow mw_B_1 = {{0}, 0, 0};

//********************************滤波器：滑动窗口中值+均值混合滤波函数*************************************//
//double MovingWindowFilter_Update(MovingWindow* mw, double new_value)
//{
//    // 添加新值到窗口
//    mw->window[mw->index] = new_value;
//    mw->index = (mw->index + 1) % WINDOW_SIZE;
//    if (mw->count < WINDOW_SIZE) mw->count++;
//    
//    // 如果窗口未满，直接返回当前值（无延迟）
//    if (mw->count < WINDOW_SIZE) {
//        return new_value;
//    }
//    
//    // 复制窗口数据进行排序（中值滤波）
//    double temp[WINDOW_SIZE];
//    memcpy(temp, mw->window, sizeof(temp));
//    
//    // 简单冒泡排序找中值
//    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
//        for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {
//            if (temp[j] > temp[j + 1]) {
//                double swap = temp[j];
//                temp[j] = temp[j + 1];
//                temp[j + 1] = swap;
//            }
//        }
//    }
//    
//    double median = temp[WINDOW_SIZE / 2]; // 中值
//    
//    // 异常值检测：如果新值与中值差异过大，认为是异常值
//    if (fabs(new_value - median) > 100) { // 阈值可根据实际情况调整
//        // 用中值替代异常值
//        new_value = median;
//        mw->window[(mw->index - 1 + WINDOW_SIZE) % WINDOW_SIZE] = median;
//    }
//    
//    // 计算均值（包括修正后的值）
//    double sum = 0;
//    for (int i = 0; i < WINDOW_SIZE; i++) {
//        sum += mw->window[i];
//    }
//    
//    return sum / WINDOW_SIZE;
//}


double MovingWindowFilter_Update(MovingWindow* mw, double new_value)
{
		uint8_t start_flag = 0;		//启动阶段标志位，从零开始电机速度迅速上升
		uint8_t zero_num = 0;
	
    // 添加新值到窗口
    mw->window[mw->index] = new_value;
    mw->index = (mw->index + 1) % WINDOW_SIZE;
    if (mw->count < WINDOW_SIZE) mw->count++;
    
    // 如果窗口未满，直接返回当前值（无延迟输出）
    if (mw->count < WINDOW_SIZE) {
        return new_value;
    }
    
		
    // 复制窗口数据便于后续处理
    double temp[WINDOW_SIZE];
    memcpy(temp, mw->window, sizeof(temp));
    
		// 计算窗口中值
    // 使用插入排序（对小数组更高效）
    for (int i = 1; i < WINDOW_SIZE; i++) {
        double key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    
    double median = temp[WINDOW_SIZE / 2];
    
		// 判断是否启动阶段
		for(int i = 0; i < WINDOW_SIZE; i++)//数窗口内0的个数
		{
			if(fabs(temp[i]) < 1.0) zero_num++;
		}
		if(zero_num > WINDOW_SIZE-2) start_flag = 1;	//0太多了，说明此时在启动阶段
		else start_flag = 0;		//非启动阶段
		
    // 异常值检测
		if(start_flag == 0)// 非启动阶段进行
		{
			if(new_value > 5000)	// 去除绝对过大的值
			{
				// 安全计算上一个索引
				int last_index = mw->index - 1;
				if (last_index < 0) last_index = WINDOW_SIZE - 1;
				
				mw->window[last_index] = 0;	//	
			}
			else
			{
				if (fabs(new_value - median) > 100) {
					// 安全计算上一个索引
					int last_index = mw->index - 1;
					if (last_index < 0) last_index = WINDOW_SIZE - 1;
					
					mw->window[last_index] = median;	//	用中值代替异常值
					new_value = median;
				}
			}
		}
    
    // 计算均值
    double sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += mw->window[i];
    }
    
    return sum / WINDOW_SIZE;
}


//double MovingWindowFilter_Update_2(MovingWindow* mw, double new_value)//二次滤波
//{
//		uint8_t start_flag = 0;		//启动阶段标志位，从零开始电机速度迅速上升
//		uint8_t zero_num = 0;
//	
//    // 添加新值到窗口
//    mw->window[mw->index] = new_value;
//    mw->index = (mw->index + 1) % WINDOW_SIZE_2;
//    if (mw->count < WINDOW_SIZE_2) mw->count++;
//    
//    // 如果窗口未满，直接返回当前值（无延迟输出）
//    if (mw->count < WINDOW_SIZE_2) {
//        return new_value;
//    }
//    
//		// 判断是否启动阶段
//		for(int i = 0; i < WINDOW_SIZE_2; i++)//数窗口内0的个数
//		{
//			if(fabs(mw->window[i]) < 1.0) zero_num++;
//		}
//		if(zero_num > WINDOW_SIZE_2-1) start_flag = 1;	//0太多了，说明此时在启动阶段
//		else start_flag = 0;		//非启动阶段
//		
//		// 异常值检测
//		if(start_flag == 0)// 非启动阶段进行
//		{
//			// 安全计算最新索引
//			int latest_index = mw->index - 1;
//			if (latest_index < 0) latest_index = WINDOW_SIZE_2 - 1;
//			// 安全计算上一个索引
//			int last_index = latest_index - 1;
//			if (last_index < 0) last_index = WINDOW_SIZE_2 - 1;
//			
//			if(fabs(mw->window[latest_index] - mw->window[last_index]) > 100)	//	突变点
//			{
//				double tem;
//				if(fabs(mw->window[latest_index]) > fabs(mw->window[last_index])) tem = mw->window[last_index];
//				else tem = mw->window[latest_index];
//				
//				return tem;
//			}
//			else return new_value;
//		}
//		else //启动阶段
//		{
//			return new_value;
//		}
//}

double MovingWindowFilter_Update_2(MovingWindow* mw, double new_value)//二次滤波
{
		uint8_t start_flag = 0;		//启动阶段标志位，从零开始电机速度迅速上升
		uint8_t zero_num = 0;
	
    // 添加新值到窗口
    mw->window[mw->index] = new_value;
    mw->index = (mw->index + 1) % WINDOW_SIZE;
    if (mw->count < WINDOW_SIZE) mw->count++;
    
    // 如果窗口未满，直接返回当前值（无延迟输出）
    if (mw->count < WINDOW_SIZE) {
        return new_value;
    }
    
		
    // 复制窗口数据便于后续处理
    double temp[WINDOW_SIZE];
    memcpy(temp, mw->window, sizeof(temp));
    
		// 计算窗口中值
    // 使用插入排序（对小数组更高效）
    for (int i = 1; i < WINDOW_SIZE; i++) {
        double key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    
    double median = temp[WINDOW_SIZE / 2];
    
		// 判断是否启动阶段
		for(int i = 0; i < WINDOW_SIZE; i++)//数窗口内0的个数
		{
			if(fabs(temp[i]) < 1.0) zero_num++;
		}
		if(zero_num > WINDOW_SIZE-2) start_flag = 1;	//0太多了，说明此时在启动阶段
		else start_flag = 0;		//非启动阶段
		
    // 异常值检测
		if(start_flag == 0)// 非启动阶段进行
		{
				if (fabs(new_value - median) > 100) {
					return median;
				}
				else return new_value;
		}
		else
			return new_value;
}


//double MovingWindowFilter_Update(MovingWindow* mw, double new_value)
//{
//		if(new_value > 2500)	new_value = 2500;	
//		else if(new_value < -2500) new_value = -2500;
//			
//		//判断是否启动阶段的变量
//		uint8_t start_flag = 0;		//启动阶段标志位，从零开始电机速度迅速上升
//		uint8_t zero_num = 0;
//	
//    // 添加新值到窗口
//    mw->window[mw->index] = new_value;
//    mw->index = (mw->index + 1) % WINDOW_SIZE;	//index进位
//    if (mw->count < WINDOW_SIZE) mw->count++;
//    
//    // 如果窗口未满，直接返回当前值（无延迟输出）
//    if (mw->count < WINDOW_SIZE) {
//        return new_value;
//    }
//    
//		// 判断是否启动阶段
//		for(int i = 0; i < WINDOW_SIZE; i++)//数窗口内0的个数
//		{
//			if(fabs(mw->window[i]) < 1.0) zero_num++;
//		}
//		if(zero_num > WINDOW_SIZE-3) start_flag = 1;	//0太多了，说明此时在启动阶段
//		else start_flag = 0;		//非启动阶段
//		
//		// 安全计算最新索引
//		int latest_index = mw->index - 1;
//		if (latest_index < 0) latest_index = WINDOW_SIZE - 1;
//		// 安全计算上一个索引
//		int last_index = latest_index - 1;
//		if (last_index < 0) last_index = WINDOW_SIZE - 1;
//		
//		if(start_flag == 0)// 非启动阶段进行
//		{
//			
//			if((new_value - mw->window[last_index])> 70 || (new_value - mw->window[last_index])<-70)	//限幅，去除异常值	
//			{
//				new_value = mw->window[last_index];	
//				mw->window[latest_index] = new_value;	//把修正后的新值放回窗口
//			}
//			
//			double filter = mw->window[last_index] * 0.7 + new_value * 0.3;
//			
//			if(filter > 2500) filter = 2500;
//			else if(filter < -2500) filter = -2500;
//			
//			return filter;
//		}
//		else	//启动阶段
//		{
////			double sum = 0;
////			for (int i = 0; i < WINDOW_SIZE; i++) {
////					sum += mw->window[i];
////			}
////			
////			mw->window[latest_index] = sum / WINDOW_SIZE;
////			
////			return mw->window[latest_index];
//			if((new_value - mw->window[last_index])> 300 || (new_value - mw->window[last_index])<-300)	//限幅，去除异常值	
//			{
//				new_value = mw->window[last_index];	
//				mw->window[latest_index] = new_value;	//把修正后的新值放回窗口
//			}
//			
//			double filter = mw->window[last_index] * 0.7 + new_value * 0.3;
//			
//			if(filter > 2500) filter = 2500;
//			else if(filter < -2500) filter = -2500;
//			
//			return filter;
//		}
//}
//********************************获取位置参数*************************************//
// 读取距离，A轮
double ReadDistance_A(void){return wheelA_dis;}
// 读取距离，B轮
double ReadDistance_B(void){return wheelB_dis;}
// 读取线数，A轮
int64_t ReadLines_A(void){return encoderA_line;}
// 读取线数，B轮
int64_t ReadLines_B(void){return encoderB_line;}

//********************************获取速度参数*************************************//
//////////////////////////////////1、获取原始速度////////////////////////////////////
// 获取A编码器速度（原始）
double GetSpeed_A(void)
{
	//暂时禁用中断并暂存变量，解决中断导致的一致性问题（此处貌似加了中断启停后数据更糟糕，所以注释掉）
	int direction_tem;
	int64_t t0_A_tem, t1_A_tem;
	//HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);  
	direction_tem = direction_A;
	t0_A_tem = t0_A;
	t1_A_tem = t1_A;
	//__HAL_GPIO_EXTI_CLEAR_IT(E1_A_Pin);//清空外部中断线中断标志位，防止启用nvic后马上进入中断
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	//换向点归零，解决换向速度数据异常
	if((direction_tem == 2) || (direction_tem == -2)) return 0;	
	
	//T取较大值，解决速度不收敛于零
	uint64_t t_now = GetTime_us();	//获取当前时刻
	uint64_t T;
	if((t_now - t1_A_tem) > (t1_A_tem - t0_A_tem))
		T = t_now - t1_A_tem;
	else
		T = t1_A_tem - t0_A_tem;
	
	//	限幅，实测最大值约为2000左右
	double speed = direction_tem * ((360.0/780.0) / (T/1000000.0));	//360.0/780.0表示编码器转一线对应轮子的度数，T/1000000.0表示一线的时间（s）
	if(speed > 2500) speed = 2500;
	else if(speed < -2500) speed = -2500;
	
	//计算轮子的角速度（度/s）
	return speed;
}

// 获取B编码器速度（原始）
double GetSpeed_B(void)
{
	//暂时禁用中断并暂存变量，解决中断导致的一致性问题（此处貌似加了中断启停后数据更糟糕，所以注释掉）
	int direction_tem;
	int64_t t0_B_tem, t1_B_tem;
	
	direction_tem = direction_B;
	t0_B_tem = t0_B;
	t1_B_tem = t1_B;
	
	//换向点归零，解决换向速度数据异常
	if((direction_tem == 2) || (direction_tem == -2)) return 0;	
	
	//T取较大值，解决速度不收敛于零
	uint64_t t_now = GetTime_us();	//获取当前时刻
	uint64_t T;
	if((t_now - t1_B_tem) > (t1_B_tem - t0_B_tem))
		T = t_now - t1_B_tem;
	else
		T = t1_B_tem - t0_B_tem;
	
	//	限幅，实测最大值约为2000左右
	double speed = direction_tem * ((360.0/780.0) / (T/1000000.0));	//360.0/780.0表示编码器转一线对应轮子的度数，T/1000000.0表示一线的时间（s）
	if(speed > 2500) speed = 2500;
	else if(speed < -2500) speed = -2500;
	
	//计算轮子的角速度（度/s）
	return speed;
}

//////////////////////////////////2、获取滤波后速度////////////////////////////////////
double speedA_last = 0;
double GetSpeed_A_filter(void)
{
		double speed_now = GetSpeed_A();
		if(speed_now>2500) speed_now = 2500;
		else if(speed_now < -2500) speed_now = -2500;
	
		double speed_1 = MovingWindowFilter_Update(&mw_A, speed_now);
		
//		//二次限幅滤波
//		//double speed_filter;
//		if((speed_1-speedA_last)> 100 || (speed_1-speedA_last)<-100)	speed_1 = speedA_last;	//限幅，去除异常值
//		//speed_filter = speedA_last*0.9 + speed_1*0.1;	//	平滑
//		speedA_last = speed_1;
	
		return MovingWindowFilter_Update_2(&mw_A_1, speed_1);
	//return speed_1;
}

double speedB_last;
double GetSpeed_B_filter(void)
{
    double speed_now = GetSpeed_B();
		if(speed_now>2500) speed_now = 2500;
		else if(speed_now < -2500) speed_now = -2500;
	
		double speed_1 = MovingWindowFilter_Update(&mw_B, speed_now);

//		return MovingWindowFilter_Update_2(&mw_B_1, speed_1);
	
//		//二次限幅滤波
//		//double speed_filter;
//		if((speed_1-speedB_last)> 100 || (speed_1-speedB_last)<-100)	speed_1 = speedB_last;	//限幅，去除异常值
//		//speed_filter = speedB_last*0.9 + speed_1*0.1;	//	平滑
//		speedB_last = speed_1;
	
		//return speed_1;
		return MovingWindowFilter_Update_2(&mw_B_1, speed_1);

}


//PS:以下两个函数存在问题：电机启动阶段由于速度变化过大被限幅处理误判，导致速度值一直是0
//// A编码器速度滤波函数
//static double speedA_last;
//double GetSpeed_A_filter(void)
//{	
//	double speed_filter;
//	double speed_now = GetSpeed_A();
//	
//	if((speed_now-speedA_last)> 50 || (speed_now-speedA_last)<-50)	speed_now = speedA_last;	//限幅，去除异常值
//	speed_filter = speedA_last*0.9 + speed_now*0.1;	//	平滑
//	speedA_last = speed_now;
//	
//	return speed_filter;
//}
//// B编码器速度滤波函数
//static double speedB_last;
//double GetSpeed_B_filter(void)
//{	
//	double speed_filter;
//	double speed_now = GetSpeed_B();
//	
//	if((speed_now-speedB_last)> 50 || (speed_now-speedB_last)<-50)	speed_now = speedB_last;	//限幅，去除异常值
//	speed_filter = speedB_last*0.9 + speed_now*0.1;	//	平滑
//	speedB_last = speed_now;
//	
//	return speed_filter;
//}

//********************************外部中断回调函数，用于编码器计数 & T法测速*************************************//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case E1_A_Pin:	//A电机编码器触发
		{
			//更新时间（测速）
			t0_A = t1_A;
			t1_A = GetTime_us();
			
			//检测正反转
			GPIO_PinState a = HAL_GPIO_ReadPin(GPIOA, E1_A_Pin);//检测A相是上升沿还是下降沿触发
			GPIO_PinState b = HAL_GPIO_ReadPin(GPIOB, E1_B_Pin);//读B相电平
			if((a == GPIO_PIN_SET && b == GPIO_PIN_SET) || (a == GPIO_PIN_RESET && b == GPIO_PIN_RESET))	//当前是正转
			{
				//更新线数
				encoderA_line++;
				
				//更新direction参数（判断刚刚结束的脉冲发生的是正转还是反转还是换向点）（测速）
				if(direction_A > 0)	{direction_A = 1;}	//上一次是正转
				else	{direction_A = 2;}								//上一次是反转
			}
			else	//当前是反转
			{
				//更新线数
				encoderA_line--;
				
				//更新direction参数（判断刚刚结束的脉冲发生的是正转还是反转还是换向点）（测速）
				if(direction_A > 0)	{direction_A = -2;}		//上一次是正转
				else	{direction_A = -1;}									//上一次是反转
			}
			
			//更新距离
			wheelA_dis = encoderA_line * DISTANCE_PER_LINE_mm;
			
			break;
		}
		case E2_A_Pin:	//B电机编码器触发
		{
			//更新时间（测速）
			t0_B = t1_B;
			t1_B = GetTime_us();
			
			//检测正反转
			GPIO_PinState a = HAL_GPIO_ReadPin(GPIOA, E2_A_Pin);//检测A相是上升沿还是下降沿触发
			GPIO_PinState b = HAL_GPIO_ReadPin(GPIOA, E2_B_Pin);//读B相电平
			if((a == GPIO_PIN_SET && b == GPIO_PIN_RESET) || (a == GPIO_PIN_RESET && b == GPIO_PIN_SET))	//正转
			{
				//更新线数
				encoderB_line++;
				
				//更新direction参数（判断刚刚结束的脉冲发生的是正转还是反转还是换向点）（测速）
				if(direction_B > 0)	{direction_B = 1;}		//上一次是正转
				else	{direction_B = 2;}									//上一次是反转
			}
			else	//反转
			{
				//更新线数
				encoderB_line--;
				
				//更新direction参数（判断刚刚结束的脉冲发生的是正转还是反转还是换向点）（测速）
				if(direction_B > 0)	{direction_B = -2;}		//上一次是正转
				else	{direction_B = -1;}									//上一次是反转
			}
			
			//更新距离
			wheelB_dis = encoderB_line * DISTANCE_PER_LINE_mm;
			
			break;
		}
	}
}


