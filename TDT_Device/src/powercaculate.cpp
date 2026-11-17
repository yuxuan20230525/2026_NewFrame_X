#include "powercaculate.h"
#include "rls.h"
using namespace Core::Control::Math; // 使用命名空间

Matrixf<2, 1> PowerTask::params;
Matrixf<2, 1> PowerTask::samples;

PowerTask powerTask;
// 创建 RLS 实例（指定维度为1）
RLS<2> rls(6e-3f, 0.99999f);

PowerTask::PowerTask():PowerManager(0.3,0.84,4,8,4,25,1){}
	
void PowerTask::init()
{
}
float  adadawdwa=0;
float  bbbbbbbb=0;
void PowerTask::run()
{
	for(int i=0;i<n;i++)
	{
		this->Tref[i]=*(powerParam->Tref+i);
		this->Wref[i]=*(powerParam->Wref+i);
		this->error[i]=*(powerParam->error+i);
		this->MotorResult[i]=*(powerParam->MotorResult+i);
	}
	effectivePower=0;
	samples[0][0]=0;
	samples[1][0]=0;
	if(powerParam->rlsEnable)
	{
		for(int i=0;i<n;i++)
		{
			effectivePower +=Tref[i]*Wref[i];
			samples[0][0]+=abs(Wref[i]);
			samples[1][0]+=Tref[i]*Tref[i];
		}
		adadawdwa=*powerParam->powerRef - effectivePower - k3;
		params     = rls.update(samples,*powerParam->powerRef - effectivePower - k3);
		k1 = fmax(params[0][0], 1e-5f);  // In case the k1 diverge to negative number
		k2 = fmax(params[1][0], 1e-5f);  // In case the k2 diverge to negative number
//		k1 = params[0][0];  // In case the k1 diverge to negative number
//		k2 = params[1][0];  // In case the k2 diverge to negative number
	}
	for(int i=0;i<n;i++)
	{
		Pcmd[i]=GetPowerCmd(Tref[i],Wref[i]);
	}
	SumErrorPcmd(error,Pcmd);
	for(int i=0;i<n;i++)
	{
		TorMax[i]=GetTorqueMax(Tref[i],Wref[i],error[i],MotorResult[i]);
	}
	bbbbbbbb=*powerParam->powerRef-Pcmd[0];
}



PowerManager::PowerManager(float k1,float k2,float k3,float kpe,float n,float Eupper,float Elower)
{
	this->k1=k1;
	this->k2=k2;
	this->k3=k3;
	this->kpe=kpe;
	this->Eupper=Eupper;
	this->Elower=Elower;
	this->n=n;
	float initParams[2] = {this->k1, this->k2};
	rls.setParamVector(Matrixf<2, 1>(initParams));
}



float PowerManager::GetTorqueMax(float t,float w,float error,float output)
{
	float pcmd=GetPowerCmd(t,w);
	float pmax=GetPowerMax(error,pcmd,powerParam->powerMaxRef);
	GetABC(t,w,pcmd,pmax,error);
	if(pcmd>pmax)
		return EquationSolution(A,B,C,(int)output);
	else 
		return output;
}



void PowerManager::SumErrorPcmd(float* error,float* Pcmd)
{
	sumError=0;
	sumPcmd=0;
	for(int i=0;i<n;i++)
	{
		 sumError+=*(error+i);
		 sumPcmd+=*(Pcmd+i);
	}
}



float PowerManager::GetPowerMax(float error,float pcmd,float* powerMaxRef)
{
	enerageError=sqrt(*powerParam->Es)-sqrt(*powerParam->Ef);
  float powerMax=*powerMaxRef-kpe*enerageError;
	powerMax=LIMIT(powerMax,0,100);
	float kcoe=0;
	if(sumError>Eupper)
		kcoe=1;
	else if(sumError<Elower)
		kcoe=0;
	else
		kcoe=(sumError-Elower)/(Eupper-Elower);
	kcoe=LIMIT(kcoe,0,1);
	float powerKp=0;
	if(sumError==0||sumPcmd==0)
	{  
		powerKp=1;
	}
	else
	 powerKp=kcoe*error/sumError+(1-kcoe)*pcmd/sumPcmd;
	powerKp=LIMIT(powerKp,0,1);
	return powerKp*powerMax;
}


float PowerManager::GetPowerCmd(float t,float w)
{
	return t*w+k1*abs(w)+k2*t*t+k3/n;
}



void PowerManager::GetABC(float t,float w,float pcmd,float power,float error)
{
	A=k2;
	B=w;
	C=k1*abs(w)+k3/n-power;
}



float PowerManager::EquationSolution(float a,float b,float c,int dir)
{
	float root1=0;
	float root2=0;
		// 计算判别式
	float discriminant = b * b - 4 * a * c;
	// 根据判别式的值判断根的情况
	if (discriminant > 0) 
	{
		// 两个不同的实根
		root1 = (-b + sqrt(discriminant)) / (2 * a);
		root2 = (-b - sqrt(discriminant)) / (2 * a);
		if(dir>=0)
			return root1>0?root1:0;
		else
			return root2<0?root2:0;
	}
	else if (discriminant == 0) 
	{
		// 一个重根
		root1 = -b / (2 * a);
   	return root1;
	}
	else 
	{
			// 没有实根
		double realPart = -b / (2 * a);
		double imaginaryPart = sqrt(-discriminant) / (2 * a);
		return realPart;
	}
}