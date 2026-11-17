#include "power_task.h"
#include "powercaculate.h"

#if USE_POWER
PidParam diaMotorInner;
PidParam diaMotorOuter;

PowerParam powerParam;
Power_Task power_Task;


void Power_Task::run()
{
	UpDate();
	powerTask.run();
//	power.useCap=1;
//	power.myPowerCanTx();
//	power.getPowerKp();
}


void Power_Task::UpDate()
{
	Es=90*20;
	Ef=power.capacitance_percentage*20;
	powerMaxRef=power.judge_information.chassisPowerLimit;
	//result=powerTask.TorMax[0]/0.000366211f;
	for(int a=0;a<4;a++)
  {
		T[a]=diaMotor->canInfo.trueCurrent*0.000366211f;
		W[a]=diaMotor->canInfo.dps*0.0087266f/19.0f;
		error[a]=ABS(motorSetSpeed-diaMotor->canInfo.speed)*6.0f*0.0087266f/19.0f;
		MotorResult[a]=diaMotor->pidInner.result*0.000366211f;
  }
}



void Power_Task::init()
{
	powerParam.Ef=&Ef;
	powerParam.Es=&Es;
	powerParam.error=error;
	powerParam.MotorResult=MotorResult;
	powerParam.powerMaxRef=&powerMaxRef;
	powerParam.powerRef=&power.nowPower;
	powerParam.Tref=T;
	powerParam.Wref=W;
	powerParam.rlsEnable=1;
	powerTask.powerParam=&powerParam;
	powerTask.init();
}

#endif





