#ifndef __DJIMOTOR_H__
#define __DJIMOTOR_H__

#include "board.h"
#include "motor.h"

class Motor : public Virtual_Motor
{
private:
	static bool IfContainSend(int can_i, int idIndex);
	static void sendCanMessage(int can_i, int idIndex);

public:
	/// 创建电机对象，并进行初始化（能进行数据接收）
	Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID);

	static void GetOriginData(CanInfo *Info, CanRxMsg *_CanRxMsg);

	/// 定时器回调函数，can发送
	static void DjisendCanMsg();

	static void DjiRecCanMsg(u8 can_x, CanRxMsg *_CanRxMsg);

	static float canBuff[2][12]; ///< 电机CAN信息缓冲区

	bool torque_control = false; // 慎用 首先只有6020可以选择是否开启电流控制，
	//	其次开启之后这一组的控制帧都会改变，要求这一组只有6020电机才可以
};

/// 结构体：电机列表，用于通过CAN口和ID检索（轮询）所有电机
typedef struct
{
	Motor *motorPoint = nullptr; ///< 电机对象指针
} MotorList;

extern MotorList motorList[2][12];

#endif