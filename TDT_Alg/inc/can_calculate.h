/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-12-04 15:37:16
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-01-25 19:14:07
 * @FilePath: \2025_NewFrame\2025_NewFram\Frame Base\Frame2025\TDT_Alg\inc\can_calculate.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CAN_CALCULATE_H__
#define __CAN_CALCULATE_H__

#include "board.h"
#pragma pack(1)
struct  CanInfo
{
	float encoder = 0;	 ///< 原始机械编码器值
	float speed = 0;		 ///< 原始机械转速
	float trueCurrent = 0; // 实际电流
	int16_t temperature = 0; // 电机温度
	u8 errCode=0;
	float Auto_totalAngle = 0.0f; // 部分电机有(大疆电机无)，反馈电机发的多圈角度

	float dps = 0.0f;				 ///< 转速（度每秒），与mpu6050单位相等
	int32_t totalAngle = 0;		 ///< 总角度
	float totalAngle_f = 0.0f;		 ///< 总角度（浮点），与mpu6050单位相等
	u8 lostFlag = 1;			 // 掉线标志位
	int32_t totalEncoder_SI = 0; // 速度积分出来的位置
	float offsetEncoder = 0;	 // 机械角度初始值
	int32_t totalRound = 0;		 ///< 总圈数
	float totalEncoder = 0;	 ///< 总角度

	float lastEncoderCalibration = 0; ///< 上次减去机械角度初值后的角度
	float encoderCalibration = 0;		///< 减去机械角度初值后的角度
	float lastEncoder = 0;			// 上次总角度值
	float lastTotalEncoder = 0;		// 上次总机械角度值
	uint16_t msgCnt = 0;				// 接受消息计数
	uint16_t lostCnt = 0;				// 接受消息计数
};

#pragma pack()

class Virtual_Motor_Calculate
{
protected:
	CanInfo *Info = nullptr;

	u8 speedUnit = 1;

	void Motor_Offset(CanInfo *Info, u8 ifsetzero);

	void rec_Calculate(CanInfo *Info, uint16_t degree = 8192); // 消息来源和输出，编码器精度

public:
	void motor_Rec_Calculate(CanInfo *Info, CanRxMsg *_CanRxMsg, uint16_t degree = 8192, u8 ifsetzero = 0);

	//virtual void GetOriginData(CanInfo *Info, CanRxMsg *_CanRxMsg) = 0; // 接收信息重写 必须要填写的是encoder和speed

	void (*GetOriginData)(CanInfo *Info, CanRxMsg *_CanRxMsg) = nullptr;
};


/** @} */

#endif
