/*
 * @Author: your name
 * @Date: 2021-05-08 23:52:16
 * @LastEditTime: 2025-01-25 19:19:05
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Infantry\Infantry_II\User\TDT_User.h
 */
#include "stdint.h"
#ifndef __TDT_INFANTRY__
#define __TDT_INFANTRY__

#define Run_Mode 1
#define Abandon !Run_Mode /**不调用的部分不编译**/

// 动作参数宏定义
#define ON 1
#define OFF 0

/**
 * @struct  _vec2f
 * @brief 二维float向量结构体
 */
struct vec2f
{
	float data[2];
};

/**
 * @struct  _vec2int16
 * @brief 二维int16向量结构体
 */
struct vec2int16
{
	short data[2];
};

/**
 * @struct  _vec3f
 * @brief 三维float向量结构体
 */
struct vec3f
{
	float data[3];
};

/**
 * @struct  _vec3int16
 * @brief 三维int16向量结构体
 */
struct vec3int16
{
	short data[3];
};

/**
 * @struct  _vec4f
 * @brief 四维float向量结构体
 */
struct vec4f
{
	float data[4];
};

/**
 * @struct  _vec4int16
 * @brief 四维int16向量结构体
 */
struct vec4int16
{
	short data[4];
};

/**
 * @struct  _vec6f
 * @brief 六维float向量结构体
 */
struct vec6f
{
	float data[6];
};

/**
 * @struct  _vec6int16
 * @brief 六维int16向量结构体
 */
struct vec6int16
{
	short data[6];
};

/**
 * @struct  _mat3f
 * @brief 3x3 float矩阵结构体
 */
struct mat3f
{
	float data[3][3];
};


#endif
