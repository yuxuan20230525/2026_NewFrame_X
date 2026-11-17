#ifndef __GUN_H__
#define __GUN_H__

#include "motor.h"
#include "pid.h"

class Gun
{
	public :
		typedef struct
		{
			bool Enable_Flag = false;
			bool Work_Flag = false;
			bool Continue_Flag;
			bool Fire_Flag = false;
		}CMD;
		void Init(void);
	    void Update(void);
	
	private :
		typedef enum 
		{
			Disbale_Waiting = 0,
			Enable_Waiting,
			Enable_Single_Firing,
			Enable_Continuous_Firing,
		}State;
		
		CMD cmd;
		State state = Disbale_Waiting;
		uint8_t Last_SWB = 0;
		
		float Other_Target;
		static constexpr uint8_t Gun_MotorCount = 3;
		Motor *motors[Gun_MotorCount] = {};
		Motor *Othermotor = {};
		PidParam Motor_tInnerParam[Gun_MotorCount];
		PidParam OtherInnerParam,OtherOuterParam;
		
		void Motor_Init(uint16_t feedbackId);
		void Clear(void);
	    void Get_CMD(void);
		void State_Change(void);
};

extern Gun gun;

#endif