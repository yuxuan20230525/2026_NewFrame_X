#ifndef _SMOOTH_H_
#define _SMOOTH_H_


// 定义滤波器结构体
struct Biquad{
    // 滤波器系数
    double b0, b1, b2;
    double a1, a2;
    // 滤波器状态
    double x1, x2; // 前两个输入样本
    double y1, y2; // 前两个输出样本
};

class NTF
{
private:
	Biquad *filter;
public:
	NTF(double fs, double notchFreq, double Q);
	double processSample(double input);
};


#endif



