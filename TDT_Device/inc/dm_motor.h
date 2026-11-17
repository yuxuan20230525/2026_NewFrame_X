/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-01-25 19:09:02
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-01-25 19:38:20
 * @FilePath: \2025_NewFrame\2025_NewFram\Frame Base\Frame2025\TDT_Device\inc\DM_Motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DM_Motor_
#define _DM_Motor_
#include "board.h"
#include "motor.h"
#include "can.h"
#define Torque_constant 0.32 // 扭矩常量
#define MAX_DMNUM 10			// 定义最大不会有10个lkmotor同时被声明

/* 参数范围 */
#define P_MIN -3.141593f// Radians   //一圈2Π
#define P_MAX  3.141593f   
#define V_MIN -20.94f    // Rad/s
#define V_MAX  20.94f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -7.0f	   
#define T_MAX  7.0f

class DM_Motor : public Virtual_Motor
{
public:
		
	DM_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t SendId,uint32_t REC_id);

	static void SendCurrentAgreement(uint8_t canbuff[8], float canResult);

	static void GetOriginData(CanInfo *Info, CanRxMsg *_CanRxMsg);

	void Motor_Send();
	// 以上为基本电机配置，下面为特殊电机的单独配置

	void mitCtrl(float p_des, float v_des, float kp, float kd, float t_ff);

	void motorEnable();

	void motorDisable();

  void saveZero();

	bool use_my_send = false;

private:
	u8 canTxBuff[8] = {0};
	
	
};

void DMList_Send();
void DMList_Receive(u8 can_x, CanRxMsg *_CanRxMsg);

extern uint8_t valid_dmmotorNum ; // 一共定义的lk电机对象
extern DM_Motor *dmList[MAX_DMNUM]; // 电机列表

#endif