#ifndef _POWERCACULATE_H_
#define _POWERCACULATE_H_

#include "math.h"
#include "task_virtual.h"
#include "Matrix.h"
#include "my_math.h"


typedef struct _PowerParam
{
	float* powerMaxRef=0;
	float* powerRef=0;
	float* Es=0;
	float* Ef=0;
	float* Tref=0;
	float* Wref=0;
	float* error=0;
	float* MotorResult=0;
	bool rlsEnable=0;
} PowerParam;




class PowerManager
{
private:
	float kpe=0;
	float Prefmax=0;
	float enerageError=0;
  float A=0;
	float B=0;
	float C=0; 
	float sumError=0;
  float sumPcmd=0;
	float Eupper=0;
	float Elower=0;
	float GetPowerMax(float error,float pcmd,float* powerMaxRef);
	void GetABC(float t,float w,float pcmd,float power,float error);
	float EquationSolution(float a,float b,float c,int dir);
public:
	float k1=0;
	float k2=0;
	float k3=0;
	float Tref[4]={0};
	float Wref[4]={0};
	float error[4]={0};
	float Pcmd[4]={0};
	float n=0;
	float MotorResult[4]={0};
	float effectivePower=0;
	PowerParam* powerParam=nullptr;
	PowerManager(float k1,float k2,float k3,float kpe,float n,float Eupper,float Elower);
	float GetPowerCmd(float t,float w);
	float GetTorqueMax(float w,float error,float pcmd,float output);
	void SumErrorPcmd(float* error,float* Pcmd);
};



class PowerTask:public PowerManager
{
private:
	static Matrixf<2, 1> samples;
  static Matrixf<2, 1> params;
public:
	PowerTask();
	float TorMax[4]={0};
	void init();
	void run();
};

extern PowerTask powerTask;





#endif



