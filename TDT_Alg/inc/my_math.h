/*****************************************************************************
File name: TDT_Alg\src\my_math.h
Description: 快速数学计算
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.16 肖银河-改写my_deathzoom函数-解决遥控器最大值会小于660问题
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "board.h"
/**
 * @ingroup TDT_ALG
 * @defgroup TDT_MY_MATH TDT_MY_MATH
 * @brief 该库包含了各种常用数学计算公式的快速解法
 * @{
 */

/// 最小线性二乘法拟合线性方程
class Linear
{
private:
	uint16_t getValueCnt = 0;
	double l[4] = {0};

public:
	float kA = 0;
	float kB = 0;
	Linear()
	{
		getValueCnt = 1;
		memset(l, 0, sizeof(l));
	}
	void clear(void)
	{
		getValueCnt = 1;
		memset(l, 0, sizeof(l));
	}
	void linearRegression(float valueX, float valueY, float valueNum, float *_aResult = 0, float *_bResult = 0, u8 resetFlag = 0)
	{
		if (resetFlag == 1)
		{
			Linear();
		}
		if (++getValueCnt < valueNum)
		{
			l[0] += valueX;
			l[1] += valueY;
			l[2] += valueX * valueY;
			l[3] += valueX * valueX;
		}
		else
		{
			kA = (valueNum * l[2] - l[0] * l[1]) / (valueNum * l[3] - l[0] * l[0]);
			kB = l[1] / valueNum - kA * l[0] / valueNum;
			if (_aResult != 0 && _bResult != 0)
			{
				*_aResult = kA;
				*_bResult = kB;
			}
		}
	}
};

/// 取绝对值
#define ABS(x) ((x) > 0 ? (x) : -(x))
/// 输出最接近 (min, max) 的x的值
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/// 输出a与b的最小值
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/// 输出a与b的最大值
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/// tan列表的最小值
#define TAN_MAP_RES 0.003921569f /* (smallest non-zero value in table) */
/// rad/°的算术值
#define RAD_PER_DEG 0.017453293f
/// tan列表的大小
#define TAN_MAP_SIZE 256
#define MY_PPPIII 3.14159f
#define MY_PPPIII_HALF 1.570796f

/// 快速正切计算
float my_atan_angle(float xx, float yy);
float my_atan_rad(float xx, float yy);
float fast_atan2(float yy, float xx);
/// 平方
float my_pow(float a);
/// 快速开方
float my_sqrt(float number);
/// 快速正弦
double my_sin_angle(float angle);
double my_sin_rad(double rad);
/// 快速余弦
double my_cos_angle(float angle);
double my_cos_rad(double rad);

/// 死区，绝对值小于zoom的xx会返回0，否则返回xx
float my_deathzoom(float xx, float zoom);
/// 6值之中返回最大的
float Math_Max(float a, float b, float c, float d, float e, float f);
/** @brief 转速转换
 * @{ */
float rpmToDps(float rpm);
float dpsToRpm(float dps);
float rpmToRadps(float rpm);
float RadpsToRpm(float radps);
/** @} */

float Ramp_function_cos(double Input, double min, double max);
float Ramp_function_sin(double Input, double min, double max);
float differangle_caculate(float angle1, float angle2);
float normalizeAngle(float angle);

// 矩阵乘法: C = A * B (A: m×n, B: n×p, C: m×p)
// 参数说明：
// - A: 输入矩阵 (按行优先存储)
// - B: 输入矩阵
// - C: 输出矩阵 (必须预先分配好内存)
// - m,n,p: 矩阵维度
void matrix_multiply(float* A, float* B, float* C, uint8_t m, uint8_t n, uint8_t p);

// 矩阵转置: B = A^T (A: m×n, B: n×m)
// 参数说明：
// - A: 输入矩阵
// - B: 输出转置矩阵 (必须预先分配好内存)
void matrix_transpose(float* A, float* B, uint8_t m, uint8_t n);

// 矩阵求逆 (3x3)
// 返回值：
// - 1: 成功
// - 0: 失败（矩阵奇异）
// 参数说明：
// - A: 输入矩阵 (3x3)
// - invA: 输出逆矩阵 (必须预先分配至少9个float空间)
uint8_t matrix_inverse3x3(float* A, float* invA);

// Cholesky分解 (3x3对称正定矩阵)
// 返回值：
// - 1: 成功
// - 0: 失败（矩阵不正定）
// 参数说明：
// - A: 输入矩阵 (3x3对称矩阵)
// - L: 输出下三角矩阵 (按行优先存储下三角部分，共6个元素)
uint8_t cholesky_decomposition3x3(float* A, float* L);

/* 线性方程组求解 */
// 高斯消元法解线性方程组 Ax = b
// 参数:
//   A: n×n 系数矩阵 (按行优先存储)
//   b: n×1 常数向量
//   x: n×1 解向量 (需预分配空间)
//   n: 矩阵维度 (最大支持255×255)
// 返回: 
//   1=成功, 0=失败(奇异矩阵或无解)
// 注意: 会修改A和b的内容，如需保留原数据请提前拷贝
uint8_t solve_linear_system(float* A, float* b, float* x, uint8_t n);


/** @} */

#endif
