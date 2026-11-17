/******************************
File name: TDT_Task\src\imu_task.cpp
Description: 陀螺仪姿态解算任务
variable :
	——————————————————————————————————————————————————————————————————————————
	 boardImu     陀螺仪类
	——————————————————————————————————————————————————————————————————————————
	 boardImu->My_data   boardImu->AHRS_data     两种解算方法的角度值
	——————————————————————————————————————————————————————————————————————————
	 imutask.forceGetOffset 置1：板载 IMU 校准零偏；HiPNUC 执行模块内部自动校平并重启主控
	——————————————————————————————————————————————————————————————————————————
	 gyroScaleFactor[3][3]    accScaleFactor[3][3]  陀螺仪旋转矩阵
	——————————————————————————————————————————————————————————————————————————
	MPU6050_def   BMI088_def   Scha634_03_def  使能某个陀螺仪
	**/
#include "imu_task.h"
#include "arm_math.h"
#include "mpu6050.h"
#include "scha634_03.h"
#include "bmi088.h"
#include "icm45686.h"
#include "TimeMatch.h"
#include "cycle.h"
#include "flash_var.h"
#include "my_math.h"
#include "dbus.h"
ImuCalc *boardImu = nullptr;
ImuCalc *boardImu2 = nullptr;
ImuCalc *boardImuBackup = nullptr; // 失效降级: 热备板载 BMI088
ImuTask imutask;
float iwatchacc[3] , iwatchgyro[3] ; //Jscope观测IMU原始数据的接口
float watchangle[3] , watchahrsangle[3] ; 
float temp=0;
ImuTask::ImuTask()
{
	status = taskStateRun;
	taskHz = _1000hz;
	forceGetOffset = 0; // 初始化为0
}

/**
 * @brief 陀螺仪任务
 * @note 负责数据读取和解算
 */

void ImuTask::run()
{
	if (this->forceGetOffset)
	{
#if (HIPNUC_def || HIPNUC_FAILOVER)
		if (boardImu == &hipnucImu)
		{
			if (!deforceFlag)
			{
				g_hipnucDbg.calibState = 4;
				g_hipnucDbg.calibFailReason = 6;
				forceGetOffset = 0;
			}
			else if (!hipnucImu.calibrateAttitude())
				forceGetOffset = 0;
		}
		else
#endif
		{
			boardImu->getOffset();
		}
	}
	if (this->ellipsoid_cali)
	{
		this->ellipsoid_fit() ;
	}
	/*读取*/
#if HIPNUC_FAILOVER
	hipnucImu.TDT_IMU_update();        // 主: 发触发/收帧/软同步
	boardImuBackup->TDT_IMU_update();  // 热备 BMI088: 一直跑保持 AHRS 收敛, 随时可接管
	boardImuBackup->imuValid = 1;       // BMI088 驱动本身未维护该通用健康标志
	failoverSupervise();               // 监督, 必要时切 boardImu
#else
	boardImu->TDT_IMU_update();
	//boardImu2->TDT_IMU_update();
#endif
	
//	memcpy(iwatchacc, boardImu->acc.accValue.data, 3 * sizeof(float));
//	memcpy(iwatchgyro, boardImu->gyro.dps.data, 3 * sizeof(float));
	watchangle[0] = boardImu->FusionAngle.yaw ;
	watchangle[1] = boardImu->FusionAngle.pitch ;
	watchangle[2] = boardImu->FusionAngle.roll ;
	
	watchahrsangle[0] = boardImu->AHRS_data.Angle.yaw ;
	watchahrsangle[1] = boardImu->AHRS_data.Angle.pitch ;
	watchahrsangle[2] = boardImu->AHRS_data.Angle.roll ;
	
	iwatchacc[0]=boardImu->acc.accValue.data[0];
	iwatchacc[1]=boardImu->acc.accValue.data[1];
	iwatchacc[2]=boardImu->acc.accValue.data[2];
	
	iwatchgyro[0]=boardImu->gyro.dps.data[0];
	iwatchgyro[1]=boardImu->gyro.dps.data[1];
	iwatchgyro[2]=boardImu->gyro.dps.data[2];
	temp=watchtemp;
}

/**
 * @brief 失效降级监督(HIPNUC_FAILOVER)
 * @note 每拍调用。HiPNUC 连续丢帧: 先转软同步(留 HiPNUC), 再不行切板载 BMI088(锁定到重启)。
 *       切 BMI088 时使用两颗 IMU 最后一次同时健康时的相对姿态做零点对齐。
 */
void ImuTask::failoverSupervise()
{
#if HIPNUC_FAILOVER
	uint32_t loss = hipnucImu.lossTicks(); // 连续丢帧拍数(1ms/拍)
	if (loss > failoverMaxLoss)
		failoverMaxLoss = loss;

	// 不能在真正切换时用“陈旧 HIP - 当前 BMI”做对齐：若失联期间机体运动，
	// 这会把真实运动误当成零偏并永久加到 BMI 上。只保存两颗 IMU 同时健康时的坐标差。
	if (loss == 0 && !hipnucImu.ahrs.initialising && !boardImuBackup->ahrs.initialising)
	{
		standbyYawOffset = hipnucImu.FusionAngle.yaw - boardImuBackup->FusionAngle.yaw;
		standbyPitchOffset = hipnucImu.FusionAngle.pitch - boardImuBackup->FusionAngle.pitch;
		standbyRollOffset = hipnucImu.FusionAngle.roll - boardImuBackup->FusionAngle.roll;
		standbyAlignmentValid = true;
	}
	switch (failState)
	{
	case 0: // 硬件同步: 触发线疑似掉 -> 转软同步, 留在 HiPNUC
		if (loss > HIPNUC_FAILOVER_SW_TICKS)
		{
			hipnucImu.switchToSoftwareSync(); // 发 ONTIME, 模块转周期自发 + 软件打戳
			swSwitchCount++;
			failState = 1;
		}
		break;
		
	case 1: // 软同步: 仍持续丢帧 -> 数据线/模块挂了, 切板载 BMI088
		if (loss > HIPNUC_FAILOVER_BMI_TICKS)
		{
			const float yawDelta = standbyAlignmentValid ? standbyYawOffset : 0.0f;
			const float pitchDelta = standbyAlignmentValid ? standbyPitchOffset : 0.0f;
			const float rollDelta = standbyAlignmentValid ? standbyRollOffset : 0.0f;
			// offset 保证后续拍坐标系一致；同时修正当拍已算完的 FusionAngle。
			boardImuBackup->yawOffset += yawDelta;
			boardImuBackup->pitchOffset += pitchDelta;
			boardImuBackup->rollOffset += rollDelta;
			boardImuBackup->FusionAngle.yaw += yawDelta;
			boardImuBackup->FusionAngle.pitch += pitchDelta;
			boardImuBackup->FusionAngle.roll += rollDelta;
			boardImu = boardImuBackup; // 接管(下游读 boardImu 自动切到 BMI088)
			imuDegraded = 1;           // 记录已由 BMI088 接管
			bmiSwitchCount++;
			failState = 2;
		}
		// 注: 软同步下若帧恢复, loss 归零, 留在软同步(不自动切回硬件触发, 防抖)
		break;
		
	case 2:
//		if(hipnucImu.offlineFlag == 0)
//		{
//			static uint8_t h_cnt = 0;
//			if(h_cnt++ > 100)
//			{
//				boardImu = &hipnucImu;
//				boardImu->yawOffset += boardImuBackup->FusionAngle.yaw - hipnucImu.FusionAngle.yaw;
//				imuDegraded = 0;
//				failState = 1;
//				h_cnt = 0;
//			}
//		}
		break;

	default: // 2: 已锁 BMI088, 保持到重启(不自动切回 HiPNUC)
		break;
	}
#endif
}

/**
 * @brief 陀螺仪初始化
 * @note 根据imu_task.h中宏定义初始化对应的陀螺仪
 */
void ImuTask::init()
{
#if HIPNUC_FAILOVER
	// 双 IMU: 主 = 外置 HiPNUC(USART1), 热备 = 板载 BMI088(SPI1)。两颗都初始化、都跑。
	boardImu = &hipnucImu;
	hipnucImu.setEulerSign(1.f, -1.f, 1.f); // 真机定标: URFR214 后轴对, pitch 角度方向反 -> 取反
	hipnucImu.setGyroSign(1.f, 1.f, 1.f);  // 配套: pitch 角速度(gyro[1])同翻, 保 PD 角度/角速度同号
	hipnucImu.setAccSign(1.f, 1.f, 1.f);   // 配套: pitch 轴 acc[1] 同翻; acc[2](垂直/落地力补)不动
	uint8_t hipnucImuMap[3] = {1, 0, 2};
	float hipnucImuMapSign[3] = {1.f, -1.f, 1.f};
	hipnucImu.setAxisRemap(hipnucImuMap, hipnucImuMapSign);
	boardImuBackup = new Bmi088(SPI1, SPI_BaudRatePrescaler_64);
	float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	float accScaleFactor[3][3]  = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	ParamToImu(boardImuBackup, gyroScaleFactor, accScaleFactor); // 备份 IMU 旋转矩阵 + flash 零偏
	hipnucImu.TDT_ImuInit();
	boardImuBackup->TDT_ImuInit();
#else
#if MPU6050_def
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	boardImu = new Mpu6050(GPIOC, GPIO_Pin_2, GPIO_Pin_1);
	float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
	float accScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
	ParamToImu(boardImu, gyroScaleFactor, accScaleFactor);
#endif
#if Scha634_03_def
	boardImu = new Scha634_03(SPI3, SPI_BaudRatePrescaler_32, CsPin(), {GPIOA, GPIO_Pin_15});
	float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	float accScaleFactor[3][3] = {{0.0f, -1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	ParamToImu(boardImu, gyroScaleFactor, accScaleFactor);
#endif
#if BMI088_def
	boardImu = new Bmi088(SPI1, SPI_BaudRatePrescaler_64);
	float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	float accScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	ParamToImu(boardImu, gyroScaleFactor, accScaleFactor);
#endif
#if	ICM45686_def
	boardImu = new Icm45686(SPI3, SPI_BaudRatePrescaler_64);
	float gyroScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	float accScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	ParamToImu(boardImu, gyroScaleFactor, accScaleFactor);
#endif
#if HIPNUC_def
	// 外置 HiPNUC HI91 (UART5 + SYNC_IN): 用全局对象(空闲中断需稳定引用), 不 new。
	// 不调 ParamToImu: 模块自带融合, 不走 int16 零偏/缩放管线。
	boardImu = &hipnucImu;
	// 轴向: 模块端 URFR 214 已把贴装(X后Y右Z上)旋正; 真机定标 pitch 方向反 -> 软件取反。
	hipnucImu.setEulerSign(1.f, -1.f, 1.f); // roll/pitch/yaw 角度符号
	hipnucImu.setGyroSign(1.f, 1.f, 1.f);  // 配套: pitch 角速度同翻
	hipnucImu.setAccSign(1.f, 1.f, 1.f);   // 配套: pitch 轴 acc[1] 同翻; acc[2] 垂直轴不动
	uint8_t hipnucImuMap[3] = {1, 0, 2};
	float hipnucImuMapSign[3] = {1.f, -1.f, 1.f};
	hipnucImu.setAxisRemap(hipnucImuMap, hipnucImuMapSign);
	//   gyro/acc 若还要轴重映射: hipnucImu.setAxisRemap({...}, {...});
#endif

	boardImu->TDT_ImuInit();
#endif // HIPNUC_FAILOVER
}

void ImuTask::ParamToImu(ImuCalc *boardImu, float GyroScaleFactor[3][3], float AccScaleFactor[3][3])
{
	boardImu->setGyroScaleFactor(GyroScaleFactor);
	boardImu->setAccScaleFactor(AccScaleFactor);
	IFlash.link(boardImu->gyro.offset, flashNum++);
	IFlash.link(boardImu->acc.offset, flashNum++);
}

/**
 * @brief 加速度计的椭球拟合
 * @note 修正加速度计各方向的量程误差
 */
u8 ImuTask::ellipsoid_fit()
{
	// 读取加速度计数据，数目等于ELLI_CNT的数值，开启debug，改变IMU姿态，静止后手动输入get_data=1
	if(!this->calib.data_ok && this->calib.get_data && this->calib.data_cnt < ELLI_CNT)
	{
		boardImu->TDT_IMU_update();
		memcpy(&this->calib.accdata[this->calib.data_cnt * 3],&boardImu->acc.accValue.data[0],3*sizeof(float)) ;
		for(int i = 0 ; i < 3 ; i++ )
		{
			this->calib.accdata[this->calib.data_cnt * 3 + i] /= 9.8f ;
		}
		this->calib.data_cnt++ ;
		this->calib.get_data = 0;
	}
	this->calib.data_cnt == ELLI_CNT ?  this->calib.data_ok = 1 , this->calib.data_cnt = 0 :  this->calib.data_ok = 0 ;
	
	if(this->calib.data_ok)
	{
		// 构建设计矩阵D和向量b
		float D[ELLI_CNT * 9];
		float b[ELLI_CNT];
		
		for(uint16_t i = 0; i < ELLI_CNT; i++) {
			float x = this->calib.accdata[i*3];
			float y = this->calib.accdata[i*3+1];
			float z = this->calib.accdata[i*3+2];
			
			D[i*9 + 0] = x*x;
			D[i*9 + 1] = y*y;
			D[i*9 + 2] = z*z;
			D[i*9 + 3] = 2*x*y;
			D[i*9 + 4] = 2*x*z;
			D[i*9 + 5] = 2*y*z;
			D[i*9 + 6] = 2*x;
			D[i*9 + 7] = 2*y;
			D[i*9 + 8] = 2*z;
			
			b[i] = 1.0f;
		}

		// 计算D^T * D 和 D^T * b
		float DT[9 * ELLI_CNT];
		matrix_transpose(D, DT, ELLI_CNT, 9);
		
		float DTD[9 * 9];
		matrix_multiply(DT, D, DTD, 9, ELLI_CNT, 9);
		
		float DTb[9];
		matrix_multiply(DT, b, DTb, 9, ELLI_CNT, 1);

		// 解线性方程组 DTD * v = DTb (使用高斯消元法)
		float v[9];
		if(!solve_linear_system(DTD, DTb, v, 9)) {
			return 0;
		}

		// 提取椭球参数
		float A = v[0];
		float B = v[1];
		float C = v[2];
		float D_ = v[3];
		float E = v[4];
		float F = v[5];
		float G = v[6];
		float H = v[7];
		float I = v[8];

		// 构建T矩阵和中心向量
		float T[9] = {A, D_, E,
					  D_, B, F,
					  E, F, C};
		
		float center_vec[3] = {G, H, I};
		
		// 计算椭球中心: center = -T^-1 * center_vec
		float invT[9];
		if(!matrix_inverse3x3(T, invT)) {
			return 0;
		}
		
		float center[3];
		matrix_multiply(invT, center_vec, center, 3, 3, 1);
		center[0] = -center[0];
		center[1] = -center[1];
		center[2] = -center[2];

		// 计算M矩阵: M = T / (center^T * T * center - 1)
		float T_center[3];
		matrix_multiply(T, center, T_center, 3, 3, 1);
		
		// 计算M矩阵: M = T / (center^T * T * center + 1)
		float centerT_T_center = 0;
		for(uint8_t i = 0; i < 3; i++) {
			centerT_T_center += center[i] * T_center[i];
		}
		float scale = 1.0f / (centerT_T_center + 1.0f); // 修改符号
		float M[9];
		for(uint8_t i = 0; i < 9; i++) {
			M[i] = T[i] * scale;
		}


		// Cholesky分解M得到L
		float L[6] = {0}; // 下三角矩阵(按行优先存储)
		if(!cholesky_decomposition3x3(M, L)) {
			return 0;
		}

		// 保存校准参数
		this->calib.scale_mat[0] = L[0];
		this->calib.scale_mat[1] = L[1];
		this->calib.scale_mat[2] = L[2];
		this->calib.scale_mat[3] = L[3];
		this->calib.scale_mat[4] = L[4];
		this->calib.scale_mat[5] = L[5];
		
		this->calib.offset[0] = center[0];
		this->calib.offset[1] = center[1];
		this->calib.offset[2] = center[2];
		
		this->ellipsoid_cali = 0 ; // 校准完成消除椭球拟合标志位
		this->calib.data_ok = 0 ;
		return 1 ;
	}
	return 0 ;
}


/**
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
