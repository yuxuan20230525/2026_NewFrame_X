/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-01-25 19:09:02
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-01-25 19:38:20
 * @FilePath: \2025_NewFrame\2025_NewFram\Frame Base\Frame2025\TDT_Device\inc\lk_motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _LK_MOTOR_
#define _LK_MOTOR_
#include "board.h"
#include "motor.h"
#include "can.h"
#define Torque_constant 0.32 // 扭矩常量
#define MAX_LKNUM 10			// 定义最大不会有10个lkmotor同时被声明
class LK_Motor : public Virtual_Motor
{
public:
	LK_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID);

	static void SendCurrentAgreement(uint8_t canbuff[8], float canResult);

	static void GetOriginData(CanInfo *Info, CanRxMsg *_CanRxMsg);

	void Motor_Send();
	// 以上为基本电机配置，下面为特殊电机的单独配置

	bool use_my_send = false;

	void motorRead_totalAngle(void); // 读电机多圈编码器

	// 只有在定义了use_my_send=1时才可以使用一下函数，否则无效
	void Auto_ctrlSpeed(float ctrlspeed); // 电机速度控制
	// 注意，以下的最大速度不能为0；
	void Auto_ctrlPosition(float ctrlposition, float maxspeed = 1000); // 多圈位置控制 ,需要搭配motorRead_totalAngle 读电机内置的角度使用

	void motorDisable(void);
	void motorStop(void);
	void motorEnable(void);
	void saveZero(void);

private:
	u8 canTxBuff[8] = {0};
	u8 readcnt = 0;
};

void LKList_Send();
void LKList_Receive(u8 can_x, CanRxMsg *_CanRxMsg);

extern uint8_t valid_lkmotorNum ; // 一共定义的lk电机对象
extern LK_Motor *lkList[MAX_LKNUM]; // 电机列表

#endif