#ifndef __SENTINEL_H__
#define __SENTINEL_H__

#include "motor.h"

class Sentinel
{
	public:
		void Init(void);
	    void Update(void);
	private:
		typedef enum 
		{
			DISABLE_WAITING = 0,
			ENABLE_WAITING,
			ENABLE_UPING,
			ENABLE_WORKING,
		}State_t;
		
		void State_Change(void);
		void StopMotor(void);
		bool Average_Speed_Update(void);
		
		static constexpr uint16_t Speed_Count = 100;
		static constexpr float F_HZ = 500.0f;
		static constexpr float dt_s = 1.0f / F_HZ;
		static constexpr float MAX_Step_S = 1000.0f / F_HZ;
		State_t Sentinel_State = DISABLE_WAITING;
		Motor *motor1;
		Motor *motor2;
		Motor *motor3;
		Motor *motor4;
		Motor *motor5;
        PidParam motor1SpeedParam {};
        float encoderValue[Speed_Count] {};
		uint8_t Count = 0;
		float CMD_Target_Speed = 0.0f;
		float CMD_End_Target_Speed = 0.0f;
		float Average_Speed = 0.0f;
};

extern Sentinel sentinel;

#endif
