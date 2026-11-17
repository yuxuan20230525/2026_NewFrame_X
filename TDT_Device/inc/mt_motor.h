#ifndef _MT_MOTOR_
#define _MT_MOTOR_
#include "board.h"
#include "motor.h"
#include "can_calculate.h"
#include "can.h"
#define MAX_MTNUM 10  //定义最大不会有10个lkmotor同时被声明

class MT_Motor:public Virtual_Motor
{
public:
	bool use_my_send;

	MT_Motor(MotorType motorType, CAN_TypeDef *_Canx,uint32_t _REC_ID);
	
	static void SendCurrentAgreement(uint8_t canbuff[8],float canResult) ;

	static void GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg);

	void Motor_Send();

	void Motor_Receive(u8 can_x,CanRxMsg *_CanRxMsg, u8 ifsetzero);//接收函数

//以上为基本电机配置，下面为特殊电机的单独配置
	
	void motorRead_totalAngle(void);  //读电机多圈编码器
	
	
		//只有在定义了use_my_send=1时才可以使用一下函数，否则无效
		void Auto_ctrlSpeed(float ctrlspeed);//电机速度控制
		//注意，以下的最大速度不能为0；
		void Auto_ctrlPosition(float ctrlposition,float maxspeed=1000);  //多圈位置控制 ,需要搭配motorRead_totalAngle 读电机内置的角度使用
			
	
	private:
	uint32_t control_id;
	u8 canTxBuff[8];
	u8 readcnt=5;
	
};

void MTList_Send();
void MTList_Receive(u8 can_x,CanRxMsg *_CanRxMsg);
extern uint8_t vallid_mtNum;   //一共定义的lk电机对象
extern MT_Motor* mtList[MAX_MTNUM];

#endif