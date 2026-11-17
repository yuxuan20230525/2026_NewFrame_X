/*****************************************************************************
File name: TDT_Task\src\imu_task.h
Description: 陀螺仪姿态解算任务
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "board.h"
#include "task_virtual.h"
#include "imu.h" 
#define MPU6050_def  0
#define Scha634_03_def 0
#define BMI088_def 1
#define ICM45686_def 0
#define ELLI_CNT 9

class ImuTask :public VirtualTask
{
public:
	ImuTask();
	void run() override;
	void init() override; 
	bool forceGetOffset=0;
	bool ellipsoid_cali = 0;
	
	void ParamToImu(ImuCalc* boardImu,float GyroScaleFactor[3][3],float AccScaleFactor[3][3]);
	u8 ellipsoid_fit() ; //加速度计的椭球拟合校准
	struct calib_temp
	{
		u8 data_ok = 0 ;
		u8 get_data = 0 ;
		u8 data_cnt = 0 ;
		float scale_mat[6];           // 缩放矩阵(下三角)
		float offset[3];              // 偏移量
		float accdata[3*ELLI_CNT] ;
	}calib ;
	
	
	
};

extern ImuCalc* boardImu;
extern ImuTask imutask;
void imuInit(void);
void Imu_Task(void);

#endif
