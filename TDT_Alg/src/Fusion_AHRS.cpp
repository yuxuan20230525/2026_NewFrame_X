/**
 * @file Fusion_AHRS.cpp
 * @author liuskywalkerjskd
 * @brief 基于加速度计和陀螺仪的AHRS算法实现
 * @note 移除了原版中所有磁力计相关代码，仅保留加速度计和陀螺仪部分
 */

#include <float.h>
#include <math.h>
#include "Fusion_AHRS.h"

MadgwickParam madgwickParam ; //梯度下降法参数结构体

//------------------------------------------------------------------------------
// 定义常量

/**
 * @brief 初始化阶段使用的增益值
 */
#define INITIAL_GAIN (10.0f)


/**
 * @brief 初始化时间
 */
#define INITIALISATION_PERIOD (2.0f)


/**
 * @brief 陀螺仪零偏校正截止频率(Hz)
 */
#define CUTOFF_FREQUENCY (0.02f)

/**
 * @brief 零偏校正超时时间(秒)
 */
#define TIMEOUT (5)

/**
 * @brief 零偏校正阈值(°/s)
 */
#define THRESHOLD (3.0f)

/**
 * @brief 是否使用梯度法补偿
 */
#define GRAD_DEC 0

//------------------------------------------------------------------------------
// 静态函数声明

/**
 * @brief 计算重力方向的一半值
 */
static inline FusionVector HalfGravity(const FusionAhrs *const ahrs);

/**
 * @brief 计算加速度计反馈
 */
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference);

/**
 * @brief 数值钳制函数
 */
static inline int Clamp(const int value, const int min, const int max);

//------------------------------------------------------------------------------
// AHRS算法函数实现
int use_grad = 0 ;
/**
 * @brief 初始化AHRS算法结构体
 * @param ahrs AHRS算法结构体指针
 */
void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,  // 默认使用NWU坐标系
            .gain = 0.5f,                      // 默认增益
            .gyroscopeRange = 4000.0f,         // 默认陀螺仪量程2000°/s
            .accelerationRejection = 32.0f,    // 默认加速度拒绝阈值30°
            .recoveryTriggerPeriod = 100,        // 默认恢复触发周期
    };
    FusionAhrsSetSettings(ahrs, &settings);
    FusionAhrsReset(ahrs);
}

/**
 * @brief 重置AHRS算法状态
 * @param ahrs AHRS算法结构体指针
 */
void FusionAhrsReset(FusionAhrs *const ahrs) {
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;  // 重置为单位四元数
    ahrs->accelerometer = FUSION_VECTOR_ZERO;       // 重置加速度计数据
		ahrs->initialising = true;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->angularRateRecovery = false;              // 重置角速率恢复标志
    ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO; // 重置加速度反馈
    ahrs->accelerometerIgnored = false;             // 重置加速度忽略标志
    ahrs->accelerationRecoveryTrigger = 0;          // 重置加速度恢复触发器
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod; // 重置超时
}

/**
 * @brief 设置AHRS算法参数
 * @param ahrs AHRS算法结构体指针
 * @param settings 参数设置结构体指针
 */
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    ahrs->settings.convention = settings->convention;
    ahrs->settings.gain = settings->gain;
    // 计算陀螺仪有效量程(留2%余量)
    ahrs->settings.gyroscopeRange = settings->gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings->gyroscopeRange;
    // 计算加速度拒绝阈值(转换为弧度并计算半角正弦的平方)
    ahrs->settings.accelerationRejection = settings->accelerationRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(FusionDegreesToRadians(settings->accelerationRejection)), 2);
    ahrs->settings.recoveryTriggerPeriod = settings->recoveryTriggerPeriod;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    
    // 如果增益为0或恢复周期为0，则禁用加速度拒绝
    if ((settings->gain == 0.0f) || (settings->recoveryTriggerPeriod == 0)) {
        ahrs->settings.accelerationRejection = FLT_MAX;
    }
		 if (ahrs->initialising == false) {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / (INITIALISATION_PERIOD);
}

/**
 * @brief 更新AHRS算法状态(不使用磁力计)
 * @param ahrs AHRS算法结构体指针
 * @param gyroscope 陀螺仪数据(°/s)
 * @param accelerometer 加速度计数据(g)
 * @param deltaTime 时间步长(秒)
 */
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime) {
    // 调用主更新函数
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, deltaTime);
}

/**
 * @brief 更新AHRS算法状态
 * @param ahrs AHRS算法结构体指针
 * @param gyroscope 陀螺仪数据(°/s)
 * @param accelerometer 加速度计数据(g)
 * @param deltaTime 时间步长(秒)
 */

void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime) {
#define Q ahrs->quaternion.element

    // 存储加速度计数据
    ahrs->accelerometer = accelerometer;

    // 如果陀螺仪数据超出量程，则重新初始化
    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) || 
        (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) || 
        (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange)) {
        const FusionQuaternion quaternion = ahrs->quaternion;
        FusionAhrsReset(ahrs);
        ahrs->quaternion = quaternion;  // 保持当前姿态
        ahrs->angularRateRecovery = true; // 设置角速率恢复标志
    }
		 // Ramp down gain during initialisation
    if (ahrs->initialising) {
			ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
			if ((ahrs->rampedGain < ahrs->settings.gain) || (ahrs->settings.gain == 0.0f))
			{
					ahrs->rampedGain = ahrs->settings.gain;
					ahrs->initialising = false;
					ahrs->angularRateRecovery = false;
			}
    }
		
		
    // 计算算法指示的重力方向(半值)
    const FusionVector halfGravity = HalfGravity(ahrs);

    // 计算加速度计反馈
    FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = true;
    if (FusionVectorIsZero(accelerometer) == false) {
        // 计算归一化加速度计数据与重力方向的反馈
        ahrs->halfAccelerometerFeedback = Feedback(FusionVectorNormalise(accelerometer), halfGravity);
        
        // 检查是否需要忽略加速度计数据
        if (((FusionVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection))) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9; // 加速恢复
        } else {
            ahrs->accelerationRecoveryTrigger += 1; // 增加触发计数
        }
        
        // 检查是否触发恢复
        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false; // 强制使用加速度计
        } else {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        
        // 钳制触发计数
        ahrs->accelerationRecoveryTrigger = Clamp(ahrs->accelerationRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);
        
        // 如果不忽略加速度计，则使用反馈
        if (ahrs->accelerometerIgnored == false) {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
        }
    }

    // 将陀螺仪数据转换为弧度/秒并缩放0.5倍
    const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));

    // 将反馈应用到陀螺仪数据
    const FusionVector adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope, FusionVectorMultiplyScalar(halfAccelerometerFeedback, ahrs->rampedGain));

    // 积分四元数变化率
    ahrs->quaternion = FusionQuaternionAdd(ahrs->quaternion, 
        FusionQuaternionMultiplyVector(ahrs->quaternion, 
            FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

    // 归一化四元数
    ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
	
	if(use_grad)
	{
		// ????????????????(??????????????????????)
		// ????????????????? q0-q3
		Madgwick_updateIMU(gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z, accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
		
		// ???????????Madgwick?????????
		float dot_product = ahrs->quaternion.element.w * madgwickParam.q0 +
							ahrs->quaternion.element.x * madgwickParam.q1 +
							ahrs->quaternion.element.y * madgwickParam.q2 +
							ahrs->quaternion.element.z * madgwickParam.q3;
		
		// ??????????????????,?? 1.0f ????????????
		float similarity_error = 1.0f - fabsf(dot_product);

		// ??????? Fusion_Power
		// ?????????????? maxFusionPower???????С???????? minFusionPower,???????
		float dynamicFusionPower = madgwickParam.minFusionPower + similarity_error * (madgwickParam.maxFusionPower - madgwickParam.minFusionPower);
		// ?????????????????
		if (dynamicFusionPower > madgwickParam.maxFusionPower) 
		{
			dynamicFusionPower = madgwickParam.maxFusionPower;
		}
			
		// ?? Madgwick ????????е?????
		// ???: filtered_value = alpha * new_value + (1 - alpha) * last_filtered_value
		madgwickParam.fq0 = madgwickParam.filterAlpha * madgwickParam.q0 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq0;
		madgwickParam.fq1 = madgwickParam.filterAlpha * madgwickParam.q1 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq1;
		madgwickParam.fq2 = madgwickParam.filterAlpha * madgwickParam.q2 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq2;
		madgwickParam.fq3 = madgwickParam.filterAlpha * madgwickParam.q3 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq3;
		
		// ???????????1???????????????
		float recipNorm = FusionFastInverseSqrt(madgwickParam.fq0 * madgwickParam.fq0 + madgwickParam.fq1 * madgwickParam.fq1 + madgwickParam.fq2 * madgwickParam.fq2 + madgwickParam.fq3 * madgwickParam.fq3);
		madgwickParam.fq0 *= recipNorm;
		madgwickParam.fq1 *= recipNorm;
		madgwickParam.fq2 *= recipNorm;
		madgwickParam.fq3 *= recipNorm;

		// ?????????????????????????
		Q.w = (1 - dynamicFusionPower) * Q.w + dynamicFusionPower * madgwickParam.fq0;
		Q.x = (1 - dynamicFusionPower) * Q.x + dynamicFusionPower * madgwickParam.fq1;
		Q.y = (1 - dynamicFusionPower) * Q.y + dynamicFusionPower * madgwickParam.fq2;
		Q.z = (1 - dynamicFusionPower) * Q.z + dynamicFusionPower * madgwickParam.fq3;
		
		// ??????????
		ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
	}
	
	
#undef Q
}

/**
 * @brief 计算重力方向的一半值
 */
static inline FusionVector HalfGravity(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu: {
            const FusionVector halfGravity = {.axis = {
                    .x = Q.x * Q.z - Q.w * Q.y,
                    .y = Q.y * Q.z + Q.w * Q.x,
                    .z = Q.w * Q.w - 0.5f + Q.z * Q.z,
            }};
            return halfGravity;
        }
        case FusionConventionNed: {
            const FusionVector halfGravity = {.axis = {
                    .x = Q.w * Q.y - Q.x * Q.z,
                    .y = -1.0f * (Q.y * Q.z + Q.w * Q.x),
                    .z = 0.5f - Q.w * Q.w - Q.z * Q.z,
            }};
            return halfGravity;
        }
    }
    return FUSION_VECTOR_ZERO;
#undef Q
}

/**
 * @brief 计算传感器反馈
 */
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference) {
    // 如果传感器与参考方向相反，则先归一化叉积结果
    if (FusionVectorDotProduct(sensor, reference) < 0.0f) {
        return FusionVectorNormalise(FusionVectorCrossProduct(sensor, reference));
    }
    return FusionVectorCrossProduct(sensor, reference);
}

/**
 * @brief 数值钳制函数
 */
static inline int Clamp(const int value, const int min, const int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 获取当前姿态四元数
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs) {
    return ahrs->quaternion;
}

/**
 * @brief 设置当前姿态四元数
 */
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion) {
    ahrs->quaternion = quaternion;
}

/**
 * @brief 获取重力向量
 */
FusionVector FusionAhrsGetGravity(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    const FusionVector gravity = {.axis = {
            .x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
            .y = 2.0f * (Q.y * Q.z + Q.w * Q.x),
            .z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
    }};
    return gravity;
#undef Q
}

/**
 * @brief 获取线性加速度(去除重力影响)
 */
FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs) {
    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu: {
            return FusionVectorSubtract(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
        }
        case FusionConventionNed: {
            return FusionVectorAdd(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
        }
    }
    return FUSION_VECTOR_ZERO;
}

/**
 * @brief 获取地球坐标系下的加速度
 */
FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
#define A ahrs->accelerometer.axis
    const float qwqw = Q.w * Q.w;
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    FusionVector accelerometer = {.axis = {
            .x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
            .y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
            .z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }};
    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu:
            accelerometer.axis.z -= 1.0f;  // 减去重力影响
            break;
        case FusionConventionNed:
            accelerometer.axis.z += 1.0f;  // 加上重力影响
            break;
    }
    return accelerometer;
#undef Q
#undef A
}

/**
 * @brief 获取AHRS内部状态
 */
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs) {
    const FusionAhrsInternalStates internalStates = {
            .accelerationError = FusionRadiansToDegrees(FusionAsin(2.0f * FusionVectorMagnitude(ahrs->halfAccelerometerFeedback))),
            .accelerometerIgnored = ahrs->accelerometerIgnored,
            .accelerationRecoveryTrigger = ahrs->settings.recoveryTriggerPeriod == 0 ? 0.0f : 
                (float) ahrs->accelerationRecoveryTrigger / (float) ahrs->settings.recoveryTriggerPeriod,
    };
    return internalStates;
}

/**
 * @brief 获取AHRS状态标志
 */
FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs) {
    const FusionAhrsFlags flags = {
            .angularRateRecovery = ahrs->angularRateRecovery,
            .accelerationRecovery = ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout,
    };
    return flags;
}

/**
 * @brief 设置航向角
 */
void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading) {
#define Q ahrs->quaternion.element
    // 计算当前偏航角
    const float yaw = atan2f(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
    // 计算当前航向与目标航向差值的一半
    const float halfYawMinusHeading = 0.5f * (yaw - FusionDegreesToRadians(heading));
    // 创建绕Z轴旋转的四元数
    const FusionQuaternion rotation = {.element = {
            .w = cosf(halfYawMinusHeading),
            .x = 0.0f,
            .y = 0.0f,
            .z = -1.0f * sinf(halfYawMinusHeading),
    }};
    // 应用旋转
    ahrs->quaternion = FusionQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}

//------------------------------------------------------------------------------
// 陀螺仪零偏校正函数

/**
 * @brief 初始化陀螺仪零偏校正
 * @param offset 校正结构体指针
 * @param sampleRate 采样率(Hz)
 */
void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate) {
    // 计算滤波器系数
    offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;  // 计算超时计数
    offset->timer = 0;                       // 重置计时器
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO; // 重置零偏估计
}

/**
 * @brief 更新陀螺仪零偏校正
 * @param offset 校正结构体指针
 * @param gyroscope 原始陀螺仪数据
 * @return 校正后的陀螺仪数据
 */
FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope) {
    // 先减去当前零偏估计
    gyroscope = FusionVectorSubtract(gyroscope, offset->gyroscopeOffset);
    
    // 如果陀螺仪数据超过阈值，重置计时器
    if ((fabsf(gyroscope.axis.x) > THRESHOLD) || 
       (fabsf(gyroscope.axis.y) > THRESHOLD) || 
       (fabsf(gyroscope.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyroscope;
    }
    
    // 如果计时器未超时，直接返回
    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyroscope;
    }
    
    // 更新零偏估计(低通滤波)
    offset->gyroscopeOffset = FusionVectorAdd(offset->gyroscopeOffset, 
        FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
    
    return gyroscope;
}

//------------------------------------------------------------------------------
// 梯度下降姿态解算
void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// 将陀螺仪的单位从 度/秒 转换为 弧度/秒
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// 根据陀螺仪的读数计算四元数的变化率
	qDot1 = 0.5f * (-madgwickParam.q1 * gx - madgwickParam.q2 * gy - madgwickParam.q3 * gz);
	qDot2 = 0.5f * (madgwickParam.q0 * gx + madgwickParam.q2 * gz - madgwickParam.q3 * gy);
	qDot3 = 0.5f * (madgwickParam.q0 * gy - madgwickParam.q1 * gz + madgwickParam.q3 * gx);
	qDot4 = 0.5f * (madgwickParam.q0 * gz + madgwickParam.q1 * gy - madgwickParam.q2 * gx);

	// 仅当加速度计读数有效时才计算反馈（避免归一化时出现 NaN）
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// 将加速度计的测量值归一化
		recipNorm = FusionFastInverseSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 辅助变量，以避免重复计算
		_2q0 = 2.0f * madgwickParam.q0;
		_2q1 = 2.0f * madgwickParam.q1;
		_2q2 = 2.0f * madgwickParam.q2;
		_2q3 = 2.0f * madgwickParam.q3;
		_4q0 = 4.0f * madgwickParam.q0;
		_4q1 = 4.0f * madgwickParam.q1;
		_4q2 = 4.0f * madgwickParam.q2;
		_8q1 = 8.0f * madgwickParam.q1;
		_8q2 = 8.0f * madgwickParam.q2;
		q0q0 = madgwickParam.q0 * madgwickParam.q0;
		q1q1 = madgwickParam.q1 * madgwickParam.q1;
		q2q2 = madgwickParam.q2 * madgwickParam.q2;
		q3q3 = madgwickParam.q3 * madgwickParam.q3;

		// 梯度下降算法的校正步骤
		// s0-s3 是目标函数（误差函数）的梯度
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * madgwickParam.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * madgwickParam.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * madgwickParam.q3 - _2q1 * ax + 4.0f * q2q2 * madgwickParam.q3 - _2q2 * ay;
		recipNorm = FusionFastInverseSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一化梯度
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		float grad_step0 = madgwickParam.beta * s0;
		float grad_step1 = madgwickParam.beta * s1;
		float grad_step2 = madgwickParam.beta * s2;
		float grad_step3 = madgwickParam.beta * s3;

		// ??????????????????????
		// ???: v_t = gamma * v_{t-1} + learning_rate * gradient
		madgwickParam.v_s0 = madgwickParam.momentum * madgwickParam.v_s0 + grad_step0;
		madgwickParam.v_s1 = madgwickParam.momentum * madgwickParam.v_s1 + grad_step1;
		madgwickParam.v_s2 = madgwickParam.momentum * madgwickParam.v_s2 + grad_step2;
		madgwickParam.v_s3 = madgwickParam.momentum * madgwickParam.v_s3 + grad_step3;

		// ?????ж????????????
		qDot1 -= madgwickParam.v_s0;
		qDot2 -= madgwickParam.v_s1;
		qDot3 -= madgwickParam.v_s2;
		qDot4 -= madgwickParam.v_s3;
		
		// 应用反馈步骤，将误差梯度反馈给四元数的变化率
		qDot1 -= madgwickParam.beta * s0;
		qDot2 -= madgwickParam.beta * s1;
		qDot3 -= madgwickParam.beta * s2;
		qDot4 -= madgwickParam.beta * s3;
	}

	// 对四元数的变化率进行积分，得到新的四元数
	madgwickParam.q0 += qDot1 * madgwickParam.invSampleFreq;
	madgwickParam.q1 += qDot2 * madgwickParam.invSampleFreq;
	madgwickParam.q2 += qDot3 * madgwickParam.invSampleFreq;
	madgwickParam.q3 += qDot4 * madgwickParam.invSampleFreq;

	// 将新的四元数归一化
	recipNorm = FusionFastInverseSqrt(madgwickParam.q0 * madgwickParam.q0 + madgwickParam.q1 * madgwickParam.q1 + madgwickParam.q2 * madgwickParam.q2 + madgwickParam.q3 * madgwickParam.q3);
	madgwickParam.q0 *= recipNorm;
	madgwickParam.q1 *= recipNorm;
	madgwickParam.q2 *= recipNorm;
	madgwickParam.q3 *= recipNorm;
}
