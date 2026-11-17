#include "FF_test_task.h"
#include "pwm.h"

//FFTestTask FFtestTask;

void FFTestTask::init()
{
	status = taskStateStop;
	taskHz = _500hz;
	
	testPidInnerParam.kp = 0.5;
	testPidInnerParam.ki = 0;
	testPidInnerParam.kd = 0;
	testPidInnerParam.integralErrorMax = 100;
	testPidInnerParam.resultMax = testMotor.getMotorCurrentLimit();
	
	testPidOuterParam.kp = 1;
	testPidOuterParam.ki = 0;
	testPidOuterParam.kd = 0;
	testPidOuterParam.integralErrorMax = 100;
	testPidOuterParam.resultMax = testMotor.getMotorSpeedLimit();
	
	testMotor.pidInner.setPlanNum(1);
	testMotor.pidInner.paramPtr = &testPidInnerParam;
	
	testMotor.pidOuter.setPlanNum(1);
	testMotor.pidOuter.paramPtr = &testPidOuterParam;
	
	testMotor.pidInner.fbValuePtr[0] = &testMotor.canInfo.speed;
	testMotor.pidOuter.fbValuePtr[0] = &testMotor.canInfo.totalAngle_f;
	
}


void FFTestTask::run()
{
	testMotor.ctrlPosition(test_a);
}

