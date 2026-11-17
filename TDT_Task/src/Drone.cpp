#include "stm32f4xx.h"                  // Device header
#include "Drone.h"
#include "my_math.h"
#include "dbus.h"

namespace 
{	
	constexpr float FullScale = 660.0f;//摇杆满量程
	constexpr float MaxSpeedScale = 1000.0f;
	constexpr float MinSpeed = 1000.0f;
	constexpr float MaxSpeed = 2000.0f;
}

float interim_Acc_Rate = 0;
Drone drone;
void Drone::Clear_All(void)
{
	 Motor_t.setCCR(MinSpeed);
}
void Drone::Get_CMD(void)//获取命令
{
	cmd.Enable_Flag = 1;//RC.Key.SW[3] == RCS::Down?true:false;
	cmd.Acc_Rate = interim_Acc_Rate*MaxSpeedScale+MinSpeed;//RC.Key.CH[1] / FullScale;
}
void Drone::Init(void)
{
	Motor_t.init();
	TIM8->RCR = 0;
	Clear_All();
	TIM_GenerateEvent(TIM8, TIM_EventSource_Update);
}
void Drone::Update(void)
{
	Get_CMD();
	State_Change();
	switch(state)
	{
		case Disbale_Waiting :
		{
			Clear_All();
			break;
		}
		case Enable_t :
		{	
			Motor_t.setCCR(cmd.Acc_Rate <= MaxSpeed?cmd.Acc_Rate: MaxSpeed);
			break;
		}
	} 
}
void Drone::State_Change(void)
{
	switch(state)
	{
		case Disbale_Waiting :
		{
//			if(dbusOfflineFlag == 0&&cmd.Enable_Flag == true)
//			{
//				state = Enable_t;
//			}
			break;
		}
		case Enable_t :
		{
//			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
//			{
//				state = Disbale_Waiting;
//			}
			break;
		}
	}
}