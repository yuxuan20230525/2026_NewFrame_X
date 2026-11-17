#include "stm32f4xx.h"                  // Device header
#include "motor.h"
#include "Steerwheel.h"  
#include "my_math.h"
#include "dbus.h"
#include "gimbal.h"

namespace 
{	
	constexpr float Steer_Max_I = 15000.0f;
	constexpr float Steer_Max_Speed = 8000.0f;
	constexpr float Wheel_Max_I = 8000.0f;
	constexpr float Zero_Encoder[4] = {4468.0f, 2277.0f, 7179.0f, 1029.0f};//steer编码器0点 372 6373
	constexpr float YawZeroAngle = -152.797852f;
	constexpr float Y_Relative_Angle_Direction = 1.0f;
	constexpr float FullScale = 660.0f;//摇杆满量程
	constexpr float Min_Feel_Cmd = 66.0f/FullScale;//死区范围
	constexpr float Max_Speed = 1000;
	constexpr float Max_W = 20;
	constexpr float DegToRad = 0.01745329252f;
	constexpr float Follow_Kp = 9.0f;      // 跟随比例增益，单位1/s
	constexpr float Follow_Ki = 2.0f;      // 未使用
}

Steer steer;

void Steer::Clear_ALL(void)
{
	for(uint8_t i = 0;i<=3;i++)
	{
		Steer[i]->pidInner.Clear();
		Steer[i]->pidOuter.Clear();
		Steer[i]->ctrlCurrent(0);
		Steer[i]->ctrlPosition(Steer[i]->canInfo.totalAngle_f);
		Wheel[i]->pidInner.Clear();
		Wheel[i]->ctrlSpeed(0);
	}
}
void Steer::Init(void)
{
	Steer[0] = new Motor(GM6020,CAN1,0x205);//顺序右上左上左下右下 和调用顺序一样放心食用 电池的反方向是正方向哈
	Steer[1] = new Motor(GM6020,CAN2,0x207);
	Steer[2] = new Motor(GM6020,CAN2,0x206);
	Steer[3] = new Motor(GM6020,CAN1,0x208);
	Wheel[0] = new Motor(M3508,CAN1,0x203);
	Wheel[1] = new Motor(M3508,CAN2,0x202);
	Wheel[2] = new Motor(M3508,CAN2,0x204);
	Wheel[3] = new Motor(M3508,CAN1,0x201);
	for(uint8_t i = 0;i<=3;i++)
	{
		Steer[i]->pidInner.paramPtr = &S_Motor_tInnerParam[i];
		Steer[i]->pidInner.fbValuePtr[0] = &Steer[i]->canInfo.speed;
		Steer[i]->pidOuter.paramPtr = &S_Motor_tOuterParam[i];
		Steer[i]->pidOuter.fbValuePtr[0] = &Steer[i]->canInfo.totalAngle_f;
		S_Motor_tInnerParam[i] = {190.0f,700.0f,0.0f,7500/700.0f,Steer_Max_I};
		S_Motor_tOuterParam[i] = {4.50f,0.0f,0.0f,0,Steer_Max_Speed};
		Steer[i]->pidInner.Clear();
		Steer[i]->pidOuter.Clear();
		Wheel[i]->pidInner.paramPtr = &W_Motor_tInnerParam[i];
		Wheel[i]->pidInner.fbValuePtr[0] = &Wheel[i]->canInfo.speed;
		W_Motor_tInnerParam[i] = {5.0f,5.0f,0.0f,100.0f/5.0f,Wheel_Max_I};
		Wheel[i]->pidInner.Clear();
		Steer[i]->setZeroValue(Zero_Encoder[i]);
		wheelFeedback.data[i] = Steer[i]->canInfo.totalAngle_f;
	}
}
void Steer::Get_CMD(void)
{
	cmd.Enable_Flag = RC.Key.SW[3] == RCS::Down?true:false;
	cmd.Model_Flag = RC.Key.SW[1] == RCS::Down?true:false;
	cmd.W_CMD_Rate = my_deathzoom(RC.Key.CH[3]/FullScale,Min_Feel_Cmd);
	cmd.Vx_CMD_Rate = my_deathzoom(-RC.Key.CH[2]/FullScale,Min_Feel_Cmd);
	cmd.W_Gimbal_Compensation = normalizeAngle(gimbal.Motor_Yaw->canInfo.totalAngle_f);
}
void Steer::State_Change(void)
{
	switch(state)
	{
		case DISABLE_Waiting :
		{
			if(dbusOfflineFlag == 0&&cmd.Enable_Flag == true)
			{
				state = ENABLE_Waiting;
			}
			break;
		}
		case ENABLE_Waiting :
		{
			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
			{
				state = DISABLE_Waiting;
			}
			else if(cmd.Model_Flag == false)
			{
				state = ENABLE_Freedom;
			}
			else if(cmd.Model_Flag == true)
			{
				state = ENABLE_Follow;
			}
			break;
		}
		case ENABLE_Follow :
		{
			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
			{
				state = DISABLE_Waiting;
			}
			else if(cmd.Model_Flag == false)
			{
				state = ENABLE_Freedom;
			}
			break;
		}
		case ENABLE_Freedom :
		{
			if(dbusOfflineFlag == 1||cmd.Enable_Flag == false)
			{
				state = DISABLE_Waiting;
			}
			else if(cmd.Model_Flag == true)
			{
				state = ENABLE_Follow;
			}
			break;
		}
	}
}

void Steer::Update_Feedback(void)
{
	for (uint8_t i = 0; i <= 3; i++)
    {
        wheelFeedback.data[i] = -Steer[i]->canInfo.totalAngle_f;
    }
}
void Steer::Apply_CMD(void)
{
	solver.fourSteeringWheel_calculate(speedDef, wheelFeedback, false);
	
	for (uint8_t i = 0; i <= 3; i++)
    {
		float wheelSpeedTarget;
		wheelSpeedTarget = solver.outputData.speed_rpm.data[i];
		float steerAngleTarget = solver.outputData.angle.data[i];
		float motorAngleNow = Steer[i]->canInfo.totalAngle_f;
		float steerAngleNow = wheelFeedback.data[i];
		float motorAngleTarget = motorAngleNow - (steerAngleTarget - steerAngleNow);
		Wheel[i]->ctrlSpeed(wheelSpeedTarget);
		Steer[i]->ctrlPosition(motorAngleTarget);		
    }
}
void Steer::Update(void)
{
	Get_CMD();
	float forwardSpeed = cmd.Vx_CMD_Rate * Max_Speed;
	float yawAngleRad = Y_Relative_Angle_Direction * cmd.W_Gimbal_Compensation * DegToRad;
	
	State_Change();
	Update_Feedback();
	switch(state)
	{
		case DISABLE_Waiting :
		{
			Clear_ALL();
			break;
		}
		case ENABLE_Waiting :
		{
			speedDef = {0,0,0};
			break;
		}
		case ENABLE_Follow :
		{
			speedDef = {forwardSpeed * cosf(yawAngleRad),-forwardSpeed * sinf(yawAngleRad),Follow_Kp * yawAngleRad};
			Apply_CMD();
			break;
		}
		case ENABLE_Freedom :
		{
			speedDef = {forwardSpeed * cosf(yawAngleRad),-forwardSpeed * sinf(yawAngleRad),cmd.W_CMD_Rate*Max_W};
			Apply_CMD();
			break;
		}
	}
}
