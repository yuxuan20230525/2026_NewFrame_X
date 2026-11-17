#include "stm32f4xx.h"                  // Device header
#include "gimbal.h"
#include "lk_motor.h"
#include "my_math.h"
#include "dbus.h"
#include "imu_task.h"

namespace 
{	
	constexpr float Pitch_Max_Speed = 60.0f ;//单位°每秒
	constexpr float Pitch_PID_Max_Speed = 70.0f;
	constexpr float Yaw_Max_Speed = 60.0f ;//单位°每秒
	constexpr float Yaw_PID_Max_Speed = 70.0f ;//单位°每秒
	
	constexpr float FullScale = 660.0f;//摇杆满量程
	constexpr float Min_Feel_Cmd = 66.0f/FullScale;//死区范围
	constexpr float kPitchMaxDeg = 62000.0f;//限位处编码器值
	constexpr float kPitchMinDeg = 6692.0f;//限位处编码器值
	constexpr float kYawMinDeg = 2200.0f;//限位处编码器值(Yaw不过零点)
	constexpr float kYawMaxDeg = 20000.0f;//限位处编码器值
	
	//单位:协议电流控制码
	constexpr float Pitch_Max_Current_Code = 1200.0f;
	constexpr float Yaw_Max_Current_Code   = 1000.0f;//最大不超过2048
	
	constexpr float dt_s = 0.002;//单位s，对应频率500HZ 
	
}
Gimbal gimbal;

volatile float watchPitchAngleError = 0.0f;
volatile float watchYawAngleError = 0.0f;//jscope观察值

void Gimbal::Protection_CMD_Deadzone(void)//死区保护
{
	if(ABS(cmd.Pitch_CMD_Speed_Rate)<Min_Feel_Cmd)
	{
		cmd.Pitch_CMD_Speed_Rate = 0;
	}
	if(ABS(cmd.Yaw_CMD_Speed_Rate)<Min_Feel_Cmd)
	{
		cmd.Yaw_CMD_Speed_Rate = 0;
	}
}
void Gimbal::Get_CMD(void)//获取命令
	{
	cmd.Flag = RC.Key.SW[3] == RCS::Down?true:false;
	cmd.Pitch_CMD_Speed_Rate = RC.Key.CH[1] / FullScale;
	cmd.Yaw_CMD_Speed_Rate = -RC.Key.CH[0] / FullScale;
}
void Gimbal::Clear(void)//清除pid
{
    Motor_Pitch->pidInner.Clear();
    Motor_Pitch->pidOuter.Clear();
    Motor_Yaw->pidInner.Clear();
    Motor_Yaw->pidOuter.Clear();
	Motor_Pitch -> ctrlCurrent(0);
	Motor_Yaw -> ctrlCurrent(0);
	Pitch_Up_Flag = Pitch_Down_Flag = true;
	Yaw_Left_Flag = Yaw_Right_Flag = true;
	Pitch_Target = boardImu->FusionAngle.pitch;
	Yaw_Target = boardImu->FusionAngle.yaw;
	
}
void Gimbal::State_Change(void)//状态转换
{
	switch(state)
	{
		case Disbale_Waiting :
		{
			if(dbusOfflineFlag == 0&&cmd.Flag == true)
			{
				Pitch_Target = boardImu->FusionAngle.pitch;
				Yaw_Target = boardImu->FusionAngle.yaw;
				state = Enable_Waiting;
			}
			break;
		}
		case Enable_Waiting :
		{
			if(cmd.Pitch_CMD_Speed_Rate != 0 ||cmd.Yaw_CMD_Speed_Rate != 0)
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
	 // Pitch 内环：角速度环
	PitchInnerParam.kp = 2.0f;
	PitchInnerParam.ki = 20.0f;
	PitchInnerParam.kd = 0.0f;
	PitchInnerParam.integralErrorMax = 50;//70.0f/PitchInnerParam.ki;
	
	// Pitch 外环：角度环
	PitchOuterParam.kp = 25.0f;
	PitchOuterParam.ki = 0.0f;
	PitchOuterParam.kd = 0.0f;
	PitchOuterParam.integralErrorMax = 0.0f;
	
	// Yaw 内环：角速度环
	YawInnerParam.kp = 16.0f;
	YawInnerParam.ki = 38.0f;
	YawInnerParam.kd = 0.0f;
	YawInnerParam.integralErrorMax = 70.0f/YawInnerParam.ki;
	YawInnerParam.positiveFBFlag = 1;
	
	// Yaw 外环：角度环
	YawOuterParam.kp = 34.0f;
	YawOuterParam.ki = 0.0f;
	YawOuterParam.kd = 0.0f;
	YawOuterParam.integralErrorMax = 0.0f;

	PitchInnerParam.resultMax = Pitch_Max_Current_Code;
    YawInnerParam.resultMax   = Yaw_Max_Current_Code;

    PitchOuterParam.resultMax = Pitch_PID_Max_Speed ;
    YawOuterParam.resultMax   = Yaw_PID_Max_Speed;

    Motor_Pitch->pidInner.paramPtr = &PitchInnerParam;
    Motor_Pitch->pidOuter.paramPtr = &PitchOuterParam;

    Motor_Yaw->pidInner.paramPtr = &YawInnerParam;
    Motor_Yaw->pidOuter.paramPtr = &YawOuterParam;

    // 外环：IMU姿态角，单位°
	Motor_Pitch->pidOuter.fbValuePtr[0] = &boardImu->FusionAngle.pitch;
	Motor_Yaw->pidOuter.fbValuePtr[0] = &boardImu->FusionAngle.yaw;
	
	// 内环：对应轴角速度，单位°/s
	Motor_Pitch->pidInner.fbValuePtr[0] = &boardImu->gyro.dps.data[1];
	Motor_Yaw->pidInner.fbValuePtr[0] = &boardImu->gyro.dps.data[2];
	Clear();
}
void Gimbal::Init(void)//云台初始化
{
	Motor_Yaw = new LK_Motor(MHF7015, CAN1, 0x141);
	Motor_Pitch = new LK_Motor(MHF7015, CAN1, 0x142);
	
	Pid_Init();
	Pitch_Target = boardImu->FusionAngle.pitch;
	Yaw_Target = boardImu->FusionAngle.yaw;
}

void Gimbal::Limit_Location(void)//限位保护
{
	const float YawEncoder = Motor_Yaw->canInfo.encoder;
	const float YawNow = boardImu->FusionAngle.yaw;
	const float PitchEncoder = Motor_Pitch->canInfo.encoder;
	const float PitchNow = boardImu->FusionAngle.pitch;
	
	if(cmd.Pitch_CMD_Speed_Rate <= 0.0f)
	{
		Pitch_Up_Flag = true;
	}
	if(cmd.Pitch_CMD_Speed_Rate >= 0.0f)
	{
		Pitch_Down_Flag = true;
	}
	if(cmd.Yaw_CMD_Speed_Rate <= 0.0f)
	{
		Yaw_Left_Flag = true;
	}
	if(cmd.Yaw_CMD_Speed_Rate >= 0.0f)
	{
		Yaw_Right_Flag = true;
	}
	
	if(Yaw_Left_Flag && YawEncoder <= kYawMinDeg && cmd.Yaw_CMD_Speed_Rate > 0.0f)
	{
		Yaw_Left_Flag = false;
		Yaw_Target = YawNow;
	}
	if(Yaw_Right_Flag && YawEncoder >= kYawMaxDeg && cmd.Yaw_CMD_Speed_Rate < 0.0f)
	{	
		Yaw_Right_Flag = false;
		Yaw_Target = YawNow;
	}
	if(Pitch_Up_Flag && PitchEncoder >= kPitchMinDeg && PitchEncoder < 33846.0f && cmd.Pitch_CMD_Speed_Rate > 0.0f)
	{
		Pitch_Up_Flag = false ;	
		Pitch_Target = PitchNow;
	}
	if(Pitch_Down_Flag && PitchEncoder <= kPitchMaxDeg && PitchEncoder >= 33846.0f && cmd.Pitch_CMD_Speed_Rate < 0.0f)
	{
		Pitch_Down_Flag = false ;	
		Pitch_Target = PitchNow;
	}
	
	if((!Pitch_Up_Flag && cmd.Pitch_CMD_Speed_Rate > 0.0f)||(!Pitch_Down_Flag && cmd.Pitch_CMD_Speed_Rate < 0.0f))
	{
		cmd.Pitch_CMD_Speed_Rate = 0.0f;
	}
	if((!Yaw_Left_Flag && cmd.Yaw_CMD_Speed_Rate > 0.0f)||(!Yaw_Right_Flag && cmd.Yaw_CMD_Speed_Rate < 0.0f))
	{
		cmd.Yaw_CMD_Speed_Rate = 0.0f;
	}
}
void Gimbal::Update_Location(void)//更新位置
{
	Limit_Location();
	
	Pitch_Target += Pitch_Max_Speed * cmd.Pitch_CMD_Speed_Rate*dt_s;
	Yaw_Target += Yaw_Max_Speed * cmd.Yaw_CMD_Speed_Rate*dt_s;
	
	const float pitchSpeedFF = Pitch_Max_Speed * cmd.Pitch_CMD_Speed_Rate;//速度前馈

	const float yawSpeedFF = Yaw_Max_Speed * cmd.Yaw_CMD_Speed_Rate;//速度前馈

	const float pitchCorrection = Motor_Pitch->pidOuter.Calculate(Pitch_Target);
	const float yawCorrection = Motor_Yaw->pidOuter.Calculate(Yaw_Target);
	
	const float pitchSpeedTarget = LIMIT(pitchSpeedFF + pitchCorrection,-Pitch_PID_Max_Speed , Pitch_PID_Max_Speed );
	const float yawSpeedTarget = LIMIT(yawSpeedFF + yawCorrection,-Yaw_PID_Max_Speed, Yaw_PID_Max_Speed);

	Motor_Pitch->pidInner.setFfValue(0,0.0f);//重力前馈 100
	
	Motor_Pitch->ctrlSpeed(pitchSpeedTarget);
	Motor_Yaw->ctrlSpeed(yawSpeedTarget);
}
void Gimbal::Update(void)//更新
{
	Get_CMD();
	Protection_CMD_Deadzone();
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
			Motor_Pitch -> ctrlPosition(Pitch_Target);
			Motor_Yaw -> ctrlPosition(Yaw_Target);
			break;
		}
		case Enable_Working :
		{
			Update_Location();
			break;
		}
	} 
	watchPitchAngleError = Pitch_Target - boardImu->FusionAngle.pitch;
	watchYawAngleError = Yaw_Target - boardImu->FusionAngle.yaw;
}
