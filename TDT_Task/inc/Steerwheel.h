#ifndef __STEERWHEEL_H__
#define __STEERWHEEL_H__

#include "fourSteeringWheel_hsh.h"

class Steer
{
	public :
		typedef struct
		{
			bool Enable_Flag = false;
			bool Model_Flag = false;
			float Vx_CMD_Rate = 0;
			float W_CMD_Rate = 0;
			float W_Gimbal_Compensation = 0;
		}CMD;
		void Init(void);//初始化
		void Update(void);//调用
	
	private :
		typedef enum 
		{
			DISABLE_Waiting,
			ENABLE_Waiting,
			ENABLE_Follow,//跟随模式 : 底盘旋转完全由云台决定
			ENABLE_Freedom,//自由模式 : 云台、底盘单独控制
		}State;
		
		vec3f speedDef = {};
		vec4f wheelFeedback = {};
		void Update_Feedback(void);    // 更新这四个舵角
		
		CMD cmd;
		FourSteeringWheel solver; 
		
		State state = DISABLE_Waiting;
		Motor*Steer[4],*Wheel[4];
		PidParam S_Motor_tInnerParam[4];
		PidParam S_Motor_tOuterParam[4];
		PidParam W_Motor_tInnerParam[4];
		
		void Get_CMD(void);//命令获取
		void Apply_CMD(void);//转换命令到电机
		void State_Change(void);//状态转换
		void Clear_ALL(void);//清除命令，保持状态
};

extern Steer steer;

#endif
