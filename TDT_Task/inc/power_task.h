#ifndef _POWER_H_
#define _POWER_H_

#include "board.h"
#include "motor.h"
#include "pid.h"
#include "task_virtual.h"
#include "imu_task.h"
#include "power.h"

class Power_Task
{
public:
	float powerMaxRef=0;
	float Es=0;
	float Ef=0;
	float T[4]={0};
	float W[4]={0};
	float error[4]={0};
	float MotorResult[4]={0};
	float motorSetSpeed=0;
	Motor *diaMotor;
	void init();
	void run() ;
	void UpDate();
};

extern Power_Task power_Task;


#endif



