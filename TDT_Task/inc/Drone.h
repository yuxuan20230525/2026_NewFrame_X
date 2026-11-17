#ifndef __DRONE_H__
#define __DRONE_H__

#include "pwm.h"

class Drone
{
	public :
		typedef struct
		{
			bool Enable_Flag = false;
			float Acc_Rate = 0.0f;
		}CMD;
		void Init(void);
	    void Update(void);
	
	private :
		typedef enum 
		{
			Disbale_Waiting = 0,
			Enable_t,
		}State;
		
		// C5 -> PC6 -> TIM8_CH1，50 Hz，每个计数为 1 us
		PWM Motor_t{TIM8, 1, GPIOC, GPIO_PinSource6,168 - 1, 20000 - 1};
		
		CMD cmd;
		State state = Enable_t;//Disbale_Waiting;
		
	    void Get_CMD(void);
		void State_Change(void);
		void Clear_All(void);
};

extern Drone drone;

#endif