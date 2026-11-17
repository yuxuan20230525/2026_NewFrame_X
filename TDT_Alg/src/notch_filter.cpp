#include "notch_filter.h"
#include "my_math.h"


/**
 *@brief 腿部状态机
 *@param fs 采样率 notchFreq 带阻中心频率 Q 品质因数
 *@note  初始化带阻滤波器，应用预畸变
 */
NTF::NTF(double fs, double notchFreq, double Q) {
    // 设计频率预畸变
    double omega_d = 2.0 * PI * notchFreq / fs; // 设计数字频率
    double omega_a = 2.0 * tan(omega_d / 2.0); // 预畸变后的模拟频率

    // 计算中间变量
    double alpha = sin(omega_a) / (2.0 * Q);
    double cosw = cos(omega_a);

    // 计算未归一化滤波器系数
    double b0 = 1.0;
    double b1_coeff = -2.0 * cosw;
    double b2 = 1.0;
    double a0 = 1.0 + alpha;
    double a1 = -2.0 * cosw;
    double a2 = 1.0 - alpha;

    // 归一化滤波器系数
    filter->b0 = b0 / a0;
    filter->b1 = b1_coeff / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
		
    // 初始化状态
    filter->x1 = 0.0;
    filter->x2 = 0.0;
    filter->y1 = 0.0;
    filter->y2 = 0.0;
}


/**
 *@brief 处理单个样本
 */
double NTF::processSample(double input) 
{
    double output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                    - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    // 更新状态
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}
