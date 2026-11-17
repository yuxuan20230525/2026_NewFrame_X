/****************************************************
调用流程：Init()初始化 ->Update()更新
*****************************************************/
#include "stm32f4xx.h"                  // Device header
#include "dbus.h" 
#include "motor.h"
#include "timer.h"
#include "my_math.h"
#include "Sentinel.h"

#define FEEL_MIN_CMD      600.0f       
#define K_REDUCTION       4
#define ALLOWED_MAX_SPEED 3.5f*915.8f //安全保护，谨慎修改
#define MAX_ASSIST_CMD    1800.0f
//#define Speed_Count       100         //平均速度的间隔为Speed_Count/2,越小响应越快，越大越稳定
//#define F_HZ              500.0f      //任务调度频率，State_Update()需放在相应频率的任务函数
//#define MAX_Step_S        1000.0f/F_HZ//最大步长/2毫秒

Sentinel sentinel;

//static float encoderValue[Speed_Count];
//static float dt_s = 1.0f/F_HZ;//2ms
//static PidParam motor1SpeedParam;
//static float CMD_Target_Speed = 0,CMD_End_Target_Speed = 0,Average_Speed = 0;
//Motor *motor1;

void Sentinel::Init(void)
{
	motor1 = new Motor(M3508, CAN1, 0x201);
	motor2 = new Motor(M3508, CAN1, 0x202);
	motor3 = new Motor(M3508, CAN1, 0x203);
	motor4 = new Motor(GM6020, CAN1, 0x20B);
	motor1SpeedParam.kp = 80.0f;
	motor1SpeedParam.ki = 110.0f;
	motor1SpeedParam.kd = 0.0f;
	motor1SpeedParam.resultMax = MAX_ASSIST_CMD;
	motor1SpeedParam.integralErrorMax = 0.1f;
	motor1->pidInner.paramPtr = &motor1SpeedParam;
	StopMotor();
}

void Sentinel::State_Change(void)//改变状态
{
	switch(Sentinel_State)
	{
		case DISABLE_WAITING :
		{
			Sentinel_State = ENABLE_WAITING;
		    break;
		}
		case ENABLE_WAITING :
		{
			if(ABS(Average_Speed) > FEEL_MIN_CMD)
			{
				Sentinel_State = ENABLE_UPING;
				CMD_End_Target_Speed = K_REDUCTION *Average_Speed;
				motor1->pidInner.Clear();
				CMD_End_Target_Speed = LIMIT(CMD_End_Target_Speed,-ALLOWED_MAX_SPEED,ALLOWED_MAX_SPEED);
				CMD_Target_Speed = motor1->canInfo.speed;
			}
			break;
		}
		case ENABLE_UPING :
		{
			if (ABS(Average_Speed) <= FEEL_MIN_CMD)
			{
				Sentinel_State = ENABLE_WAITING;
			}
			else if(ABS(CMD_Target_Speed-CMD_End_Target_Speed) <= 0.1f)
			{
				CMD_Target_Speed = CMD_End_Target_Speed;
				Sentinel_State = ENABLE_WORKING;
			}
			
			break;
		}
		case ENABLE_WORKING :
		{
			if(ABS(Average_Speed) <= FEEL_MIN_CMD)
			{
				Sentinel_State = ENABLE_WAITING;
			}
			break;
		}
	}
}

void Sentinel::StopMotor(void)
{
	CMD_Target_Speed = 0;
	CMD_End_Target_Speed = 0;
	motor1->pidInner.Clear();
	motor1->ctrlCurrent(0);
}

bool Sentinel::Average_Speed_Update()//更新平均速度
{
//	static uint8_t Count = 0;
	float Sum = 0;
	if (Count<Speed_Count)
	{
		encoderValue[Count] = motor1->canInfo.totalEncoder;
		Count++;
		motor1->ctrlCurrent(0);
		return false ;
	}
	else
		{
			for(uint8_t j = 0;j<Speed_Count-1;j++)
			{
				encoderValue[j]=encoderValue[j+1];
			}
		
			encoderValue[Speed_Count-1] = motor1->canInfo.totalEncoder;
			for(uint8_t k = 0;k<Speed_Count/2;k++)
			{
				Sum += (encoderValue[k+Speed_Count/2]-encoderValue[k]);
			}	
		
			Average_Speed = Sum*60/(8192*Speed_Count*Speed_Count*dt_s/4);
			return true ;
		}
}
void Sentinel::Update(void)
{
	if(Average_Speed_Update())
	{
		State_Change();
		
		switch(Sentinel_State)
		{
			case DISABLE_WAITING :
			{
				StopMotor();
			    break;
			}
			case ENABLE_WAITING :
			{
				StopMotor();
				break;
			}			
			case ENABLE_UPING :
			{
				const float maxStep = MAX_Step_S;
				const float delta = CMD_End_Target_Speed - CMD_Target_Speed;
				CMD_Target_Speed += LIMIT(delta, -maxStep, maxStep);

				motor1->ctrlSpeed(CMD_Target_Speed);
				break;
			}	
			case ENABLE_WORKING :
			{
				motor1->ctrlSpeed(CMD_Target_Speed);
				break;
			}		
		}
	}
}
