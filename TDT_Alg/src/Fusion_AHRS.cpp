/**
 * @file Fusion_AHRS.cpp
 * @author liuskywalkerjskd
 * @brief ���ڼ��ٶȼƺ������ǵ�AHRS�㷨ʵ��
 * @note �Ƴ���ԭ�������д�������ش��룬���������ٶȼƺ������ǲ���
 */

#include <float.h>
#include <math.h>
#include "Fusion_AHRS.h"

MadgwickParam madgwickParam ; //�ݶ��½��������ṹ��

//------------------------------------------------------------------------------
// ���峣��

/**
 * @brief ��ʼ���׶�ʹ�õ�����ֵ
 */
#define INITIAL_GAIN (10.0f)


/**
 * @brief ��ʼ��ʱ��
 */
#define INITIALISATION_PERIOD (2.0f)


/**
 * @brief ��������ƫУ����ֹƵ��(Hz)
 */
#define CUTOFF_FREQUENCY (0.02f)

/**
 * @brief ��ƫУ����ʱʱ��(��)
 */
#define TIMEOUT (5)

/**
 * @brief ��ƫУ����ֵ(��/s)
 */
#define THRESHOLD (3.0f)

/**
 * @brief �Ƿ�ʹ���ݶȷ�����
 */
#define GRAD_DEC 0

//------------------------------------------------------------------------------
// ��̬��������

/**
 * @brief �������������һ��ֵ
 */
static inline FusionVector HalfGravity(const FusionAhrs *const ahrs);

/**
 * @brief ������ٶȼƷ���
 */
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference);

/**
 * @brief ��ֵǯ�ƺ���
 */
static inline int Clamp(const int value, const int min, const int max);

//------------------------------------------------------------------------------
// AHRS�㷨����ʵ��
int use_grad = 0 ;
/**
 * @brief ��ʼ��AHRS�㷨�ṹ��
 * @param ahrs AHRS�㷨�ṹ��ָ��
 */
void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,  // Ĭ��ʹ��NWU����ϵ
            .gain = 0.5f,                      // Ĭ������
            .gyroscopeRange = 4000.0f,         // Ĭ������������2000��/s
            .accelerationRejection = 32.0f,    // Ĭ�ϼ��ٶȾܾ���ֵ30��
            .recoveryTriggerPeriod = 100,        // Ĭ�ϻָ���������
            .restGain = 3.0f,                    // rest detection boost gain
            .restGyroscopeThreshold = 5.0f,      // rest: gyro < 5 deg/s
            .restAccelerationMin = 0.9f,         // rest: acc > 0.9g
            .restAccelerationMax = 1.1f,         // rest: acc < 1.1g
            .restDebouncePeriod = 50,            // rest: 50 samples debounce
            .dynamicFeedbackScale = 50.0f,       // dynamic feedback scale factor
    };
    FusionAhrsSetSettings(ahrs, &settings);
    FusionAhrsReset(ahrs);
}

/**
 * @brief ����AHRS�㷨״̬
 * @param ahrs AHRS�㷨�ṹ��ָ��
 */
void FusionAhrsReset(FusionAhrs *const ahrs) {
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;  // ����Ϊ��λ��Ԫ��
    ahrs->accelerometer = FUSION_VECTOR_ZERO;       // ���ü��ٶȼ�����
		ahrs->initialising = true;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->angularRateRecovery = false;              // ���ý����ʻָ���־
    ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO; // ���ü��ٶȷ���
    ahrs->accelerometerIgnored = false;             // ���ü��ٶȺ��Ա�־
    ahrs->accelerationRecoveryTrigger = 0;          // ���ü��ٶȻָ�������
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod; // ���ó�ʱ
    ahrs->restCounter = 0;
    ahrs->atRest = false;
}

/**
 * @brief ����AHRS�㷨����
 * @param ahrs AHRS�㷨�ṹ��ָ��
 * @param settings �������ýṹ��ָ��
 */
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    ahrs->settings.convention = settings->convention;
    ahrs->settings.gain = settings->gain;
    // ������������Ч����(��2%����)
    ahrs->settings.gyroscopeRange = settings->gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings->gyroscopeRange;
    // ������ٶȾܾ���ֵ(ת��Ϊ���Ȳ����������ҵ�ƽ��)
    ahrs->settings.accelerationRejection = settings->accelerationRejection == 0.0f ? FLT_MAX : powf(0.5f * sinf(FusionDegreesToRadians(settings->accelerationRejection)), 2);
    ahrs->settings.recoveryTriggerPeriod = settings->recoveryTriggerPeriod;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    
    // �������Ϊ0��ָ�����Ϊ0������ü��ٶȾܾ�
    if ((settings->gain == 0.0f) || (settings->recoveryTriggerPeriod == 0)) {
        ahrs->settings.accelerationRejection = FLT_MAX;
    }
		 if (ahrs->initialising == false) {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / (INITIALISATION_PERIOD);
    ahrs->settings.restGain = settings->restGain > 0.0f ? settings->restGain : 3.0f;
    // Rest detection thresholds (store as squared for comparison)
    const float gyroThr = settings->restGyroscopeThreshold >= 0.0f ? settings->restGyroscopeThreshold : 5.0f;
    ahrs->settings.restGyroscopeThreshold = gyroThr * gyroThr;
    const float accMin = settings->restAccelerationMin >= 0.0f ? settings->restAccelerationMin : 0.9f;
    ahrs->settings.restAccelerationMin = accMin * accMin;
    const float accMax = settings->restAccelerationMax > 0.0f ? settings->restAccelerationMax : 1.1f;
    ahrs->settings.restAccelerationMax = accMax * accMax;
    ahrs->settings.restDebouncePeriod = settings->restDebouncePeriod > 0 ? settings->restDebouncePeriod : 50;
    ahrs->settings.dynamicFeedbackScale = settings->dynamicFeedbackScale >= 0.0f ? settings->dynamicFeedbackScale : 50.0f;
}

/**
 * @brief ����AHRS�㷨״̬(��ʹ�ô�����)
 * @param ahrs AHRS�㷨�ṹ��ָ��
 * @param gyroscope ����������(��/s)
 * @param accelerometer ���ٶȼ�����(g)
 * @param deltaTime ʱ�䲽��(��)
 */
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime) {
    // ���������º���
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, deltaTime);
}

/**
 * @brief ����AHRS�㷨״̬
 * @param ahrs AHRS�㷨�ṹ��ָ��
 * @param gyroscope ����������(��/s)
 * @param accelerometer ���ٶȼ�����(g)
 * @param deltaTime ʱ�䲽��(��)
 */

void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime) {
#define Q ahrs->quaternion.element

    // �洢���ٶȼ�����
    ahrs->accelerometer = accelerometer;

    // ������������ݳ������̣������³�ʼ��
    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) || 
        (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) || 
        (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange)) {
        const FusionQuaternion quaternion = ahrs->quaternion;
        FusionAhrsReset(ahrs);
        ahrs->quaternion = quaternion;  // ���ֵ�ǰ��̬
        ahrs->angularRateRecovery = true; // ���ý����ʻָ���־
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

    // Rest detection: boost gain when stationary for faster convergence
    if (!ahrs->initialising) {
        const float gyroMagSq = FusionVectorMagnitudeSquared(gyroscope);
        const float accMagSq = FusionVectorMagnitudeSquared(accelerometer);
        // gyro < threshold AND acc within [min, max]
        if (gyroMagSq < ahrs->settings.restGyroscopeThreshold && accMagSq > ahrs->settings.restAccelerationMin && accMagSq < ahrs->settings.restAccelerationMax) {
            if (ahrs->restCounter < (int)ahrs->settings.restDebouncePeriod) {
                ahrs->restCounter++;
            } else {
                ahrs->atRest = true;
                ahrs->rampedGain = ahrs->settings.restGain;
            }
        } else {
            ahrs->restCounter = 0;
            if (ahrs->atRest) {
                ahrs->atRest = false;
                ahrs->rampedGain = ahrs->settings.gain;
            }
        }
    }

    // �����㷨ָʾ����������(��ֵ)
    const FusionVector halfGravity = HalfGravity(ahrs);

    // ������ٶȼƷ���
    FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = true;
    if (FusionVectorIsZero(accelerometer) == false) {
        // �����һ�����ٶȼ���������������ķ���
        ahrs->halfAccelerometerFeedback = Feedback(FusionVectorNormalise(accelerometer), halfGravity);
        
        // ����Ƿ���Ҫ���Լ��ٶȼ�����
        // 初始化阶段跳过加速度拒绝，无条件采信
        if (ahrs->initialising) {
            ahrs->accelerometerIgnored = false;
        }
        else if (((FusionVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection))) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 3;
        } else {
            ahrs->accelerationRecoveryTrigger += 1;
        }
        
        // ����Ƿ񴥷��ָ�
        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false; // ǿ��ʹ�ü��ٶȼ�
        } else {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        
        // ǯ�ƴ�������
        ahrs->accelerationRecoveryTrigger = Clamp(ahrs->accelerationRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);
        
        // Apply feedback with dynamic scaling based on acceleration magnitude
        if (ahrs->accelerometerIgnored == false) {
            // |a| near 1g → scale≈1; |a| far from 1g → scale→0
            const float accMag = FusionVectorMagnitude(accelerometer);
            const float accError = fabsf(accMag - 1.0f);
            const float feedbackScale = 1.0f / (1.0f + ahrs->settings.dynamicFeedbackScale * accError * accError);
            halfAccelerometerFeedback = FusionVectorMultiplyScalar(ahrs->halfAccelerometerFeedback, feedbackScale);
        }
    }

    // ������������ת��Ϊ����/�벢����0.5��
    const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));

    // ������Ӧ�õ�����������
    const FusionVector adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope, FusionVectorMultiplyScalar(halfAccelerometerFeedback, ahrs->rampedGain));

    // ������Ԫ���仯��
    ahrs->quaternion = FusionQuaternionAdd(ahrs->quaternion, 
        FusionQuaternionMultiplyVector(ahrs->quaternion, 
            FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

    // ��һ����Ԫ��
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
		// ?????????????? maxFusionPower???????��???????? minFusionPower,???????
		float dynamicFusionPower = madgwickParam.minFusionPower + similarity_error * (madgwickParam.maxFusionPower - madgwickParam.minFusionPower);
		// ?????????????????
		if (dynamicFusionPower > madgwickParam.maxFusionPower) 
		{
			dynamicFusionPower = madgwickParam.maxFusionPower;
		}
			
		// ?? Madgwick ????????��?????
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
 * @brief �������������һ��ֵ
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
 * @brief ���㴫��������
 */
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference) {
    // �����������ο������෴�����ȹ�һ��������
    if (FusionVectorDotProduct(sensor, reference) < 0.0f) {
        return FusionVectorNormalise(FusionVectorCrossProduct(sensor, reference));
    }
    return FusionVectorCrossProduct(sensor, reference);
}

/**
 * @brief ��ֵǯ�ƺ���
 */
static inline int Clamp(const int value, const int min, const int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief ��ȡ��ǰ��̬��Ԫ��
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs) {
    return ahrs->quaternion;
}

/**
 * @brief ���õ�ǰ��̬��Ԫ��
 */
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion) {
    ahrs->quaternion = quaternion;
}

/**
 * @brief ��ȡ��������
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
 * @brief ��ȡ���Լ��ٶ�(ȥ������Ӱ��)
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
 * @brief ��ȡ��������ϵ�µļ��ٶ�
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
            accelerometer.axis.z -= 1.0f;  // ��ȥ����Ӱ��
            break;
        case FusionConventionNed:
            accelerometer.axis.z += 1.0f;  // ��������Ӱ��
            break;
    }
    return accelerometer;
#undef Q
#undef A
}

/**
 * @brief ��ȡAHRS�ڲ�״̬
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
 * @brief ��ȡAHRS״̬��־
 */
FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs) {
    const FusionAhrsFlags flags = {
            .angularRateRecovery = ahrs->angularRateRecovery,
            .accelerationRecovery = ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout,
            .atRest = ahrs->atRest,
    };
    return flags;
}

/**
 * @brief ���ú����
 */
void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading) {
#define Q ahrs->quaternion.element
    // ���㵱ǰƫ����
    const float yaw = atan2f(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
    // ���㵱ǰ������Ŀ�꺽���ֵ��һ��
    const float halfYawMinusHeading = 0.5f * (yaw - FusionDegreesToRadians(heading));
    // ������Z����ת����Ԫ��
    const FusionQuaternion rotation = {.element = {
            .w = cosf(halfYawMinusHeading),
            .x = 0.0f,
            .y = 0.0f,
            .z = -1.0f * sinf(halfYawMinusHeading),
    }};
    // Ӧ����ת
    ahrs->quaternion = FusionQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}

//------------------------------------------------------------------------------
// ��������ƫУ������

/**
 * @brief ��ʼ����������ƫУ��
 * @param offset У���ṹ��ָ��
 * @param sampleRate ������(Hz)
 */
void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate) {
    // �����˲���ϵ��
    offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;  // ���㳬ʱ����
    offset->timer = 0;                       // ���ü�ʱ��
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO; // ������ƫ����
}

/**
 * @brief ������������ƫУ��
 * @param offset У���ṹ��ָ��
 * @param gyroscope ԭʼ����������
 * @return У���������������
 */
FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope) {
    // �ȼ�ȥ��ǰ��ƫ����
    gyroscope = FusionVectorSubtract(gyroscope, offset->gyroscopeOffset);
    
    // ������������ݳ�����ֵ�����ü�ʱ��
    if ((fabsf(gyroscope.axis.x) > THRESHOLD) || 
       (fabsf(gyroscope.axis.y) > THRESHOLD) || 
       (fabsf(gyroscope.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyroscope;
    }
    
    // �����ʱ��δ��ʱ��ֱ�ӷ���
    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyroscope;
    }
    
    // ������ƫ����(��ͨ�˲�)
    offset->gyroscopeOffset = FusionVectorAdd(offset->gyroscopeOffset, 
        FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
    
    return gyroscope;
}

//------------------------------------------------------------------------------
// �ݶ��½���̬����
void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// �������ǵĵ�λ�� ��/�� ת��Ϊ ����/��
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// ���������ǵĶ���������Ԫ���ı仯��
	qDot1 = 0.5f * (-madgwickParam.q1 * gx - madgwickParam.q2 * gy - madgwickParam.q3 * gz);
	qDot2 = 0.5f * (madgwickParam.q0 * gx + madgwickParam.q2 * gz - madgwickParam.q3 * gy);
	qDot3 = 0.5f * (madgwickParam.q0 * gy - madgwickParam.q1 * gz + madgwickParam.q3 * gx);
	qDot4 = 0.5f * (madgwickParam.q0 * gz + madgwickParam.q1 * gy - madgwickParam.q2 * gx);

	// �������ٶȼƶ�����Чʱ�ż��㷴���������һ��ʱ���� NaN��
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// �����ٶȼƵĲ���ֵ��һ��
		recipNorm = FusionFastInverseSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// �����������Ա����ظ�����
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

		// �ݶ��½��㷨��У������
		// s0-s3 ��Ŀ�꺯�������������ݶ�
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * madgwickParam.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * madgwickParam.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * madgwickParam.q3 - _2q1 * ax + 4.0f * q2q2 * madgwickParam.q3 - _2q2 * ay;
		recipNorm = FusionFastInverseSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // ��һ���ݶ�
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

		// ?????��????????????
		qDot1 -= madgwickParam.v_s0;
		qDot2 -= madgwickParam.v_s1;
		qDot3 -= madgwickParam.v_s2;
		qDot4 -= madgwickParam.v_s3;
		
		// Ӧ�÷������裬������ݶȷ�������Ԫ���ı仯��
		qDot1 -= madgwickParam.beta * s0;
		qDot2 -= madgwickParam.beta * s1;
		qDot3 -= madgwickParam.beta * s2;
		qDot4 -= madgwickParam.beta * s3;
	}

	// ����Ԫ���ı仯�ʽ��л��֣��õ��µ���Ԫ��
	madgwickParam.q0 += qDot1 * madgwickParam.invSampleFreq;
	madgwickParam.q1 += qDot2 * madgwickParam.invSampleFreq;
	madgwickParam.q2 += qDot3 * madgwickParam.invSampleFreq;
	madgwickParam.q3 += qDot4 * madgwickParam.invSampleFreq;

	// ���µ���Ԫ����һ��
	recipNorm = FusionFastInverseSqrt(madgwickParam.q0 * madgwickParam.q0 + madgwickParam.q1 * madgwickParam.q1 + madgwickParam.q2 * madgwickParam.q2 + madgwickParam.q3 * madgwickParam.q3);
	madgwickParam.q0 *= recipNorm;
	madgwickParam.q1 *= recipNorm;
	madgwickParam.q2 *= recipNorm;
	madgwickParam.q3 *= recipNorm;
}
