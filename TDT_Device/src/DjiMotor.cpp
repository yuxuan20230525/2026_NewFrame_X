#include "DjiMotor.h"
#include "can.h"
#include "dbus.h"

MotorList motorList[2][12] = {0};
float Motor::canBuff[2][12] = {0};

extern uint8_t deforceFlag;

Motor::Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID) : Virtual_Motor(motorType, _Canx,  _Std_ID , _Std_ID)
{
	if (_Std_ID < 0x200 || _Std_ID > 0x20C)
	{
		enableFlag.disable = 1;
		return;
	}
	
	if(_Std_ID <= 0x204)
		motorInfo.Send_ID =0x200;
	else if(_Std_ID <= 0x208 )
		motorInfo.Send_ID =0x1ff;
	else 
		motorInfo.Send_ID =0x2ff;
	
	
	if (motorList[(u8)motorInfo.can_x][_Std_ID - 0x201].motorPoint != nullptr && motorList[(u8)motorInfo.can_x][_Std_ID - 0x201].motorPoint != this)
	{
		motorList[(u8)motorInfo.can_x][_Std_ID - 0x201].motorPoint->enableFlag.disable = 1;
		enableFlag.disable = 1;
		return;
	}
	motorList[(u8)motorInfo.can_x][_Std_ID - 0x201].motorPoint = this;
	
	motor_Calculate.GetOriginData=&GetOriginData; //接受函数传递
}

void Motor::DjisendCanMsg()
{
	for (int can_i = 0; can_i < 2; can_i++)
		for (int idIndex = 0; idIndex < 3; idIndex++)
			for (int idOffset = 0; idOffset < 4; idOffset++)
			{
				Motor *Motor = motorList[can_i][idIndex * 4 + idOffset].motorPoint;
				if (Motor == 0)
					continue;
				Motor->motorRun();
				canBuff[can_i][idIndex * 4 + idOffset] = Motor->canResult;
			}

	for (int can_i = 0; can_i < 2; can_i++)
		for (int idIndex = 0; idIndex < 3; idIndex++)
		{
			if (IfContainSend(can_i, idIndex))
			{
				sendCanMessage(can_i, idIndex);
			}
		}
}

bool Motor::IfContainSend(int can_i, int idIndex)
{
	bool containMotor = false;

	for (int idOffset = 0; idOffset < 4; idOffset++)
	{
		Motor *Motor = motorList[can_i][idIndex * 4 + idOffset].motorPoint;

		if (Motor == 0 || Motor->enableFlag.disable || Motor->enableFlag.justRead)
		{
			continue;
		}

		containMotor = true;
	}
	return containMotor;
}

void Motor::sendCanMessage(int can_i, int idIndex)
{
	int canId = (idIndex == 0) ? 0x200 : (idIndex == 1) ? 0x1ff
														: 0x2ff;

	for (int idOffset = 0; idOffset < 4; idOffset++)
	{
		if (motorList[can_i][idIndex * 4 + idOffset].motorPoint->torque_control == 1 && motorList[can_i][idIndex * 4 + idOffset].motorPoint->getType() == GM6020)
			canId = (idIndex == 0) ? 0x200 : (idIndex == 1) ? 0x1FE
															: 0x2FE;
	}
	canTx(&canBuff[can_i][idIndex * 4], (can_i == 0) ? CAN1 : CAN2, canId);
}


void Motor::GetOriginData(CanInfo *Info, CanRxMsg *_CanRxMsg)
{
	Info->encoder = (int16_t)((_CanRxMsg->Data[0] << 8) | (_CanRxMsg->Data[1]));	 // 机械角度
	Info->speed = (int16_t)((_CanRxMsg->Data[2] << 8) | (_CanRxMsg->Data[3]));		 // 速度
	Info->trueCurrent = (int16_t)((_CanRxMsg->Data[4] << 8) | (_CanRxMsg->Data[5])); // 实际电流
	Info->temperature = (int16_t)(_CanRxMsg->Data[6]);
}

void Motor::DjiRecCanMsg(u8 can_x, CanRxMsg *_CanRxMsg)
{
		for (int can_i = 0; can_i < 2; can_i++)
		for (int idIndex = 0; idIndex < 3; idIndex++)
			for (int idOffset = 0; idOffset < 4; idOffset++)
			{
				Motor *Motor = motorList[can_i][idIndex * 4 + idOffset].motorPoint;
				if (Motor == 0)
					continue;
				Motor->motorReceive(_CanRxMsg,can_x,Motor->getType()!=GM6020);
			
			}
}
