#include "stm32f4xx.h"                  // Device header
#include "gimbal.h"
#include "motor.h"
#include "my_math.h"
#include "dbus.h"
#include "imu_task.h"

namespace 
{	
	constexpr float Yaw_Max_Speed = 200.0f ;//单位°每秒
	constexpr float Yaw_PID_Max_Speed = 600.0f ;//单位°每秒
	constexpr float FullScale = 660.0f;//摇杆满量程
	constexpr float Min_Feel_Cmd = 66.0f/FullScale;//死区范围
	constexpr float Yaw_Max_Current_Code = 10000.0f;//最大不超过16384
	constexpr float dt_s = 0.002;//单位s，对应频率500HZ
	
	constexpr float YawZeroEncoder = 4715.0f;
}

Gimbal gimbal;

void Gimbal::Get_CMD(void)//获取命令
	{
	cmd.Flag = RC.Key.SW[3] == RCS::Down?true:false;
	cmd.Chassis_W = 0.0f;
	cmd.Yaw_CMD_Speed_Rate = my_deathzoom(-RC.Key.CH[0] / FullScale,Min_Feel_Cmd);
    if (boardImu != nullptr)
    {
        cmd.Chassis_W = boardImu->gyro.dps.data[2];
    }
}
void Gimbal::Clear_ALL(void)//清除pid
{
    Motor_Yaw->pidInner.Clear();
    Motor_Yaw->pidOuter.Clear();
	Motor_Yaw -> ctrlCurrent(0);
	Yaw_Target = Motor_Yaw->canInfo.totalAngle_f;
}
void Gimbal::State_Change(void)//状态转换
{
	switch(state)
	{
		case Disbale_Waiting :
		{
			if(dbusOfflineFlag == 0&&cmd.Flag == true)
			{
				Yaw_Target = Motor_Yaw->canInfo.totalAngle_f;
				state = Enable_Waiting;
			}
			break;
		}
		case Enable_Waiting :
		{
			if(cmd.Yaw_CMD_Speed_Rate != 0)
			{
				state = Enable_Working;
			}
			if(dbusOfflineFlag == 1||gimbal.cmd.Flag == false)
			{
				state = Disbale_Waiting;
			}
			break;
		}
		case Enable_Working :
		{
			if(dbusOfflineFlag == 1||cmd.Flag == false)
			{
				state = Disbale_Waiting;
			}
			break;
		}
	}
}

void Gimbal::Pid_Init(void)//pid初始化
{
	YawInnerParam = {8.0f,100.0f,0.0f,5000/100.0f,Yaw_Max_Current_Code};
	YawOuterParam = {120.0f,0.0f,0.01f,0,Yaw_PID_Max_Speed};
    Motor_Yaw->pidInner.paramPtr = &YawInnerParam;
    Motor_Yaw->pidOuter.paramPtr = &YawOuterParam;
    Motor_Yaw->pidOuter.fbValuePtr[0] = &Motor_Yaw->canInfo.totalAngle_f;
	Motor_Yaw->pidInner.fbValuePtr[0] = &Motor_Yaw->canInfo.dps;
	Clear_ALL();
}
void Gimbal::Init(void)//云台初始化
{
	Motor_Yaw = new Motor(GM6020, CAN2, 0x205);
	Pid_Init();
	Motor_Yaw->setZeroValue(YawZeroEncoder);
	Yaw_Target = Motor_Yaw->canInfo.totalAngle_f;
}

void Gimbal::Update_Location(void)//更新位置
{
	Yaw_Target += (Yaw_Max_Speed * cmd.Yaw_CMD_Speed_Rate - cmd.Chassis_W) * dt_s;
	const float yawSpeedFF = Yaw_Max_Speed * cmd.Yaw_CMD_Speed_Rate- cmd.Chassis_W;//速度前馈
	const float yawCorrection = Motor_Yaw->pidOuter.Calculate(Yaw_Target);
	const float yawSpeedTarget = LIMIT(yawSpeedFF + yawCorrection,-Yaw_PID_Max_Speed, Yaw_PID_Max_Speed);
	Motor_Yaw->ctrlSpeed(yawSpeedTarget);
}
void Gimbal::Update(void)//更新
{
	Get_CMD();
	State_Change();
	switch(state)
	{
		case Disbale_Waiting :
		{
			Clear_ALL();
			break;
		}
		case Enable_Waiting:
		case Enable_Working:
		{
			Update_Location();
			break;
		}
	} 
}
