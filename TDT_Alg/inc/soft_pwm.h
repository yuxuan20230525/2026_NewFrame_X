#ifndef _SOFT_PWM_H_
#define _SOFT_PWM_H_

#include "board.h"
#include "bmi088.h"
#include "pid.h"

class soft_pwm
{
public:
	soft_pwm(uint16_t TempVal);
	void tempCtrlInit();
	void Simulate_PWM();

private:
	int runTime = 0;
	uint16_t SetTemp = 0;
	int16_t Op_Rate = 0; // 输出率，模拟占空比
	void Get_Temp();
	void PC5_Init();
	void Calc_Pid();
	Pid tempCtrlPid;
	float tempfb = 0.0f;
	PidParam tempPidPara;
};

extern soft_pwm bmiTempCtrl;

#endif