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
#include "hipnuc_imu.h"
// 选某一个陀螺仪。外置 HiPNUC 与板载互斥: 用 HiPNUC 时 HIPNUC_def 置 1、其余置 0。
#define MPU6050_def  0
#define Scha634_03_def 0
#define BMI088_def 1
#define ICM45686_def 0
#define HIPNUC_def 0  // 外置 HiPNUC HI91 (USART1/4-pin口 + SYNC_IN), 与板载 BMI088 互斥, 见 hipnuc_imu.h
// 失效降级: 主 HiPNUC + 热备板载 BMI088 同时初始化, 线掉自动降级。
// 置 1 时独立于上面互斥宏(忽略 BMI088_def/HIPNUC_def), 双 IMU 都跑。
#define HIPNUC_FAILOVER 0
#define HIPNUC_FAILOVER_SW_TICKS  0  // 连续丢帧多少拍(1ms)→转软同步(发 ONTIME, 留 HiPNUC)
#define HIPNUC_FAILOVER_BMI_TICKS 2000  // 连续丢帧多少拍→切板载 BMI088(锁定到重启)
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

	/* 失效降级(HIPNUC_FAILOVER): 监督 HiPNUC 健康, 必要时切软同步/切 BMI088 */
	void failoverSupervise();
	bool imuDegraded = 0;   ///< 已降级到板载 BMI088 的状态/遥测标志
	uint8_t failState = 0;  ///< 0=HW同步 1=软同步 2=已锁BMI088
	uint32_t failoverMaxLoss = 0; ///< 监督器观测到的历史最大连续丢帧拍数
	uint32_t swSwitchCount = 0;
	uint32_t bmiSwitchCount = 0;
	bool standbyAlignmentValid = false;
	float standbyYawOffset = 0.0f;   ///< 两颗 IMU 最后一次同时健康时的姿态差
	float standbyPitchOffset = 0.0f;
	float standbyRollOffset = 0.0f;
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
extern ImuCalc* boardImuBackup; ///< 失效降级备份 IMU(板载 BMI088); 非 failover build 为 nullptr
extern ImuTask imutask;
void imuInit(void);
void Imu_Task(void);

#endif
