/******************************
File name: TDT_Task\src\imu_task.cpp
Description: 陀螺仪姿态解算任务
variable :
	——————————————————————————————————————————————————————————————————————————
	 boardImu     陀螺仪类
	——————————————————————————————————————————————————————————————————————————
	 boardImu->My_data   boardImu->AHRS_data     两种解算方法的角度值
	——————————————————————————————————————————————————————————————————————————
	 imutask.forceGetOffset 置1则校准陀螺仪并重启主控
	——————————————————————————————————————————————————————————————————————————
	 gyroScaleFactor[3][3]    accScaleFactor[3][3]  陀螺仪旋转矩阵
	——————————————————————————————————————————————————————————————————————————
	MPU6050_def   BMI088_def   Scha634_03_def  使能某个陀螺仪
	**/
#include "imu_task.h"
#include "mpu6050.h"
#include "scha634_03.h"
#include "bmi088.h"
#include "icm45686.h"
#include "TimeMatch.h"
#include "cycle.h"
#include "flash_var.h"
#include "my_math.h"
extern TimeSimultaneity imuTimeMatch;
ImuCalc *boardImu = nullptr;
ImuTask imutask;
float iwatchacc[3] , iwatchgyro[3] ; //Jscope观测IMU原始数据的接口
float watchangle[3] , watchahrsangle[3] ; 
float temp=0;
ImuTask::ImuTask()
{
	status = taskStateRun;
	taskHz = _500hz;
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
		boardImu->getOffset();
	}
	if (this->ellipsoid_cali)
	{
		this->ellipsoid_fit() ;
	}
	/*读取*/
	boardImu->TDT_IMU_update();
	//boardImu2->TDT_IMU_update();
	
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
 * @brief 陀螺仪初始化
 * @note 根据imu_task.h中宏定义初始化对应的陀螺仪
 */
void ImuTask::init()
{
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

	boardImu->TDT_ImuInit();
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
