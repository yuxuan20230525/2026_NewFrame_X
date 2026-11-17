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
			float Chassis_W = 0.0f;
		}CMD;
		void Init(void);
	    void Update(void);
		Motor *Motor_Yaw;
	
	private :
		typedef enum 
		{
			Disbale_Waiting = 0,
			Enable_Waiting,
			Enable_Working,
		}State;
		
		PidParam YawInnerParam{};
		PidParam YawOuterParam{};
		
    	CMD cmd;
		State state = Disbale_Waiting;
		
    	float Yaw_Target = 0;
		
		void Pid_Init(void);
		void Clear_ALL(void);
	    void Get_CMD(void);
		void State_Change(void);
		void Update_Location(void);
		void Limit_Location(void);
};

extern Gimbal gimbal;

#endif
