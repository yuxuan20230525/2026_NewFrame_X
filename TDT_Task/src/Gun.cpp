#include "stm32f4xx.h"                  // Device header
#include "Gun.h"
#include "my_math.h"
#include "dbus.h"

namespace 
{	
	constexpr float Gun_Max_I = 8000.0f;
	constexpr float Other_Max_I = 16384.0f;
	constexpr float Other_PID_Max_Speed = 16000.0f;
	
	constexpr float Fire_Angle = 3480.5f;
	constexpr float Fire_Direction = 1;
}

Gun gun;

volatile float TargetDeg   = 0.0f; // 目标位置
volatile float AngleDeg    = 0.0f; // 实际位置
volatile float ErrorDeg    = 0.0f; // 位置误差
volatile float TargetRpm   = 0.0f; // 内环目标转速
volatile float SpeedRpm    = 0.0f; // 实际转速
volatile float CurrentCode = 0.0f; // 限幅后的电流指令码
volatile float OuterP = 0;
volatile float OuterI = 0;
void Gun::Get_CMD(void)//获取命令
{
	cmd.Enable_Flag = RC.Key.SW[3] == RCS::Down?true:false;
	cmd.Work_Flag = RC.Key.SW[0] == RCS::Down?true:false;
	cmd.Continue_Flag = RC.Key.SW[1] == RCS::Down?true:false;
	const uint8_t Now_SWB = RC.Key.SW[2];
	
	cmd.Fire_Flag = !dbusOfflineFlag && Last_SWB != RCS::Lost && Now_SWB != RCS::Lost && Now_SWB != Last_SWB;

	Last_SWB = dbusOfflineFlag ? 0 : Now_SWB;
}

void Gun::Clear()
{
	for(int8_t i = 0;i<Gun_MotorCount;i++)
	{
	motors[i]->ctrlCurrent(0);
	motors[i]->pidInner.Clear();
	}
	Othermotor->ctrlCurrent(0);
	Other_Target = Othermotor->canInfo.totalAngle_f;
}
void Gun::Motor_Init(uint16_t feedbackId)
{
	for(int8_t i = 0;i<Gun_MotorCount;i++)
	{
	motors[i] = new Motor(M2006, CAN1, feedbackId+(uint16_t)i);
    motors[i]->pidInner.paramPtr = &Motor_tInnerParam[i];
    motors[i]->pidInner.fbValuePtr[0] = &motors[i]->canInfo.speed;
	
	Motor_tInnerParam[i].kp = 3.0f;
	Motor_tInnerParam[i].ki = 22.0f;
	Motor_tInnerParam[i].kd = 0.0f;
	Motor_tInnerParam[i].integralErrorMax = 50/Motor_tInnerParam[i].ki;
	
	Motor_tInnerParam[i].resultMax = Gun_Max_I;
	motors[i]->pidInner.Clear();
	}
}
void Gun::Init(void)
{
	Motor_Init(0x201);
	Othermotor = new Motor(M2006, CAN2, 0x202);
	Othermotor->pidInner.paramPtr = &OtherInnerParam;
	Othermotor->pidInner.fbValuePtr[0] = &Othermotor->canInfo.speed;
	Othermotor->pidOuter.paramPtr = &OtherOuterParam;
	Othermotor->pidOuter.fbValuePtr[0] = &Othermotor->canInfo.totalAngle_f;
	
	OtherInnerParam.kp = 4.50f;
	OtherInnerParam.ki = 5.0f;
	OtherInnerParam.kd = 0.0f;
	OtherInnerParam.integralErrorMax = 100.0f/OtherInnerParam.ki;
	
	OtherOuterParam.kp = 5.0f;
	OtherOuterParam.ki = 0.0f;
	OtherOuterParam.kd = 0.0f;
	OtherOuterParam.integralErrorMax = 10000.0f;
	
	OtherInnerParam.resultMax = Other_Max_I;
    OtherOuterParam.resultMax = Other_PID_Max_Speed;
	
	Other_Target = Othermotor->canInfo.totalAngle_f;
	
	Othermotor->ctrlPosition(Other_Target);
	
}
void Gun::State_Change(void)//状态转换
{
	switch(state)
	{
		case Disbale_Waiting :
		{
			if(dbusOfflineFlag == 0&&cmd.Enable_Flag == true)
			{
				state = Enable_Waiting;
			}
			break;
		}
		case Enable_Waiting :
		{
			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
			{
				state = Disbale_Waiting;
			}
			else if(cmd.Work_Flag == true&&cmd.Continue_Flag == false)
			{
				state = Enable_Single_Firing;
			}
			else if(cmd.Work_Flag == true&&cmd.Continue_Flag == true)
			{
				state = Enable_Continuous_Firing;
			}
			break;
		}
		case Enable_Single_Firing :
		{
			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
			{
				state = Disbale_Waiting;
			}
			else if(cmd.Work_Flag==false)
			{
				state = Enable_Waiting;
			}
			else if(cmd.Continue_Flag == true)
			{
				state = Enable_Continuous_Firing;
			}
			break;
		}
		
		case Enable_Continuous_Firing:
		{
			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
			{
				state = Disbale_Waiting;
			}
			else if(cmd.Work_Flag==false)
			{
				state = Enable_Waiting;
			}
			else if(cmd.Continue_Flag == false)
			{
				state = Enable_Single_Firing;
			}
			break;
		}
	}
}

void Gun::Update(void)//更新
{
	Get_CMD();
	State_Change();
	
	switch(state)
	{
		case Disbale_Waiting :
		{
			Clear();
			break;
		}
		case Enable_Waiting :
		{	
			for(int8_t i = 0;i<Gun_MotorCount;i++)
			{
				motors[i]->ctrlSpeed(0);
			}
			Othermotor->ctrlPosition(Other_Target);
			break;
		}
		case Enable_Single_Firing :
		{
			for(int8_t i = 0;i<Gun_MotorCount;i++)
			{
				motors[i]->ctrlSpeed(i == 2?-8000:8000);
			}
			if(cmd.Fire_Flag == true)
			{
				Other_Target -=Fire_Direction*Fire_Angle;
			}
//			Othermotor->ctrlSpeed(Other_Target);
			Othermotor->ctrlPosition(Other_Target);
			break;
		}
		case Enable_Continuous_Firing :
		{
			for(int8_t i = 0;i<Gun_MotorCount;i++)
			{
				motors[i]->ctrlSpeed(i == 2?-8000:8000);
			}
	
			Other_Target -=Fire_Direction*Fire_Angle/25.0f;
			
			Othermotor->ctrlPosition(Other_Target);
			break;
		}
	}
	
	TargetDeg   = Other_Target;
	AngleDeg    = Othermotor->canInfo.totalAngle_f;
	ErrorDeg    = TargetDeg - AngleDeg;
	TargetRpm   = Othermotor->pidOuter.result;
	SpeedRpm    = Othermotor->canInfo.speed;
	CurrentCode = Othermotor->canResult;
	OuterP = OtherOuterParam.kp * Othermotor->pidOuter.error;
	OuterI = OtherOuterParam.ki * Othermotor->pidOuter.integralError;
}
