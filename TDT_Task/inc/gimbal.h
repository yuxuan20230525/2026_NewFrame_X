#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "motor.h"
#include "pid.h"

class Gimbal
{
	public :
		typedef struct
		{
			bool  Flag = false;
			float Yaw_CMD_Speed_Rate = 0.0f;
			float Pitch_CMD_Speed_Rate = 0.0f;
		}CMD;
		void Init(void);
	    void Update(void);
	
	private :
		typedef enum 
		{
			Disbale_Waiting = 0,
			Enable_Waiting,
			Enable_Working,
		}State;
		PidParam PitchInnerParam{};
		PidParam PitchOuterParam{};
		PidParam YawInnerParam{};
		PidParam YawOuterParam{};
		
    	CMD cmd;
		State state = Disbale_Waiting;
		
	    LK_Motor *Motor_Yaw;
		LK_Motor *Motor_Pitch;
		
		float Pitch_Target = 0 ;
    	float Yaw_Target = 0;
		
		//限位保护标志			
		bool Yaw_Left_Flag = true;
		bool Yaw_Right_Flag = true;
		bool Pitch_Up_Flag = true;
		bool Pitch_Down_Flag = true;
			
		void Pid_Init(void);
		void Clear(void);
	    void Get_CMD(void);
		void State_Change(void);
		void Update_Location(void);
		void Protection_CMD_Deadzone(void);
		void Limit_Location(void);

};

extern Gimbal gimbal;

#endif