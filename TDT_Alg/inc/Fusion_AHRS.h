/**
 * @file Fusion_AHRS.h
 * @author liuskywalkerjskd
 * @brief ���ڼ��ٶȼƺ������ǵ�AHRS�㷨ʵ��
 * @note �Ƴ���ԭ�������д�������ش��룬���������ٶȼƺ������ǲ���
 */

#ifndef FUSION_H
#define FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// ��ѧ�ⶨ��

/**
 * @brief 3D�����ṹ��
 */
typedef union {
    float array[3];
    struct {
        float x;
        float y;
        float z;
    } axis;
} FusionVector;

/**
 * @brief ��Ԫ���ṹ��
 */
typedef union {
    float array[4];
    struct {
        float w;  // ʵ��
        float x;  // i�鲿
        float y;  // j�鲿
        float z;  // k�鲿
    } element;
} FusionQuaternion;

/**
 * @brief 3x3���󣨰�������洢��
 */
typedef union {
    float array[3][3];
    struct {
        float xx; float xy; float xz;
        float yx; float yy; float yz;
        float zx; float zy; float zz;
    } element;
} FusionMatrix;

/**
 * @brief ŷ���ǽṹ��
 * @note ��ת(roll)������(pitch)��ƫ��(yaw)�ֱ��Ӧ��X��Y��Z�����ת
 */
typedef union {
    float array[3];
    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;
} FusionEuler;

/**
 * @brief ����������
 */
#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {0.0f, 0.0f, 0.0f} })

/**
 * @brief ��λ��������
 */
#define FUSION_VECTOR_ONES ((FusionVector){ .array = {1.0f, 1.0f, 1.0f} })

/**
 * @brief ��λ��Ԫ������
 */
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){ .array = {1.0f, 0.0f, 0.0f, 0.0f} })

/**
 * @brief ��λ������
 */
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){ .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}} })

/**
 * @brief ��ŷ���ǳ���
 */
#define FUSION_EULER_ZERO ((FusionEuler){ .array = {0.0f, 0.0f, 0.0f} })

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

//------------------------------------------------------------------------------
// ��ѧ����������

/**
 * @brief ��ת����
 */
static inline float FusionDegreesToRadians(const float degrees) {
    return degrees * ((float) M_PI / 180.0f);
}

/**
 * @brief ����ת��
 */
static inline float FusionRadiansToDegrees(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

/**
 * @brief ��ȫ�ķ����Һ���
 */
static inline float FusionAsin(const float value) {
    if (value <= -1.0f) return (float) M_PI / -2.0f;
    if (value >= 1.0f) return (float) M_PI / 2.0f;
    return asinf(value);
}

/**
 * @brief ���ٷ�ƽ�������㣨�����㷨��
 */
static inline float FusionFastInverseSqrt(const float x) {
    typedef union { float f; int32_t i; } Union32;
    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

/**
 * @brief �ж������Ƿ�Ϊ0
 */
static inline bool FusionVectorIsZero(const FusionVector vector) {
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

/**
 * @brief �����ӷ�
 */
static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x + vectorB.axis.x,
        .y = vectorA.axis.y + vectorB.axis.y,
        .z = vectorA.axis.z + vectorB.axis.z,
    }};
}

/**
 * @brief ��������
 */
static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x - vectorB.axis.x,
        .y = vectorA.axis.y - vectorB.axis.y,
        .z = vectorA.axis.z - vectorB.axis.z,
    }};
}

/**
 * @brief �������������
 */
static inline float FusionVectorSum(const FusionVector vector) {
    return vector.axis.x + vector.axis.y + vector.axis.z;
}

/**
 * @brief ���������˷�
 */
static inline FusionVector FusionVectorMultiplyScalar(const FusionVector vector, const float scalar) {
    return (FusionVector){.axis = {
        .x = vector.axis.x * scalar,
        .y = vector.axis.y * scalar,
        .z = vector.axis.z * scalar,
    }};
}

/**
 * @brief ������Ԫ�س˷���Hadamard����
 */
static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x * vectorB.axis.x,
        .y = vectorA.axis.y * vectorB.axis.y,
        .z = vectorA.axis.z * vectorB.axis.z,
    }};
}

/**
 * @brief �������
 */
static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.y * vectorB.axis.z - vectorA.axis.z * vectorB.axis.y,
        .y = vectorA.axis.z * vectorB.axis.x - vectorA.axis.x * vectorB.axis.z,
        .z = vectorA.axis.x * vectorB.axis.y - vectorA.axis.y * vectorB.axis.x,
    }};
}

/**
 * @brief �������
 */
static inline float FusionVectorDotProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return FusionVectorSum(FusionVectorHadamardProduct(vectorA, vectorB));
}

/**
 * @brief ����ģ��ƽ��
 */
static inline float FusionVectorMagnitudeSquared(const FusionVector vector) {
    return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

/**
 * @brief ����ģ
 */
static inline float FusionVectorMagnitude(const FusionVector vector) {
    return sqrtf(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief ������һ��
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
    const float magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
    return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}

/**
 * @brief ��Ԫ���ӷ�
 */
static inline FusionQuaternion FusionQuaternionAdd(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
    return (FusionQuaternion){.element = {
        .w = quaternionA.element.w + quaternionB.element.w,
        .x = quaternionA.element.x + quaternionB.element.x,
        .y = quaternionA.element.y + quaternionB.element.y,
        .z = quaternionA.element.z + quaternionB.element.z,
    }};
}

/**
 * @brief ��Ԫ���˷�
 */
static inline FusionQuaternion FusionQuaternionMultiply(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
    return (FusionQuaternion){.element = {
        .w = quaternionA.element.w * quaternionB.element.w - quaternionA.element.x * quaternionB.element.x - quaternionA.element.y * quaternionB.element.y - quaternionA.element.z * quaternionB.element.z,
        .x = quaternionA.element.w * quaternionB.element.x + quaternionA.element.x * quaternionB.element.w + quaternionA.element.y * quaternionB.element.z - quaternionA.element.z * quaternionB.element.y,
        .y = quaternionA.element.w * quaternionB.element.y - quaternionA.element.x * quaternionB.element.z + quaternionA.element.y * quaternionB.element.w + quaternionA.element.z * quaternionB.element.x,
        .z = quaternionA.element.w * quaternionB.element.z + quaternionA.element.x * quaternionB.element.y - quaternionA.element.y * quaternionB.element.x + quaternionA.element.z * quaternionB.element.w,
    }};
}

/**
 * @brief ��Ԫ���������˷�
 */
static inline FusionQuaternion FusionQuaternionMultiplyVector(const FusionQuaternion quaternion, const FusionVector vector) {
    return (FusionQuaternion){.element = {
        .w = -quaternion.element.x * vector.axis.x - quaternion.element.y * vector.axis.y - quaternion.element.z * vector.axis.z,
        .x = quaternion.element.w * vector.axis.x + quaternion.element.y * vector.axis.z - quaternion.element.z * vector.axis.y,
        .y = quaternion.element.w * vector.axis.y - quaternion.element.x * vector.axis.z + quaternion.element.z * vector.axis.x,
        .z = quaternion.element.w * vector.axis.z + quaternion.element.x * vector.axis.y - quaternion.element.y * vector.axis.x,
    }};
}

/**
 * @brief ��Ԫ����һ��
 */
static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion) {
    const float magnitudeReciprocal = FusionFastInverseSqrt(
        quaternion.element.w * quaternion.element.w + 
        quaternion.element.x * quaternion.element.x + 
        quaternion.element.y * quaternion.element.y + 
        quaternion.element.z * quaternion.element.z);
    return (FusionQuaternion){.element = {
        .w = quaternion.element.w * magnitudeReciprocal,
        .x = quaternion.element.x * magnitudeReciprocal,
        .y = quaternion.element.y * magnitudeReciprocal,
        .z = quaternion.element.z * magnitudeReciprocal,
    }};
}

/**
 * @brief �����������˷�
 */
static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector) {
    return (FusionVector){.axis = {
        .x = matrix.element.xx * vector.axis.x + matrix.element.xy * vector.axis.y + matrix.element.xz * vector.axis.z,
        .y = matrix.element.yx * vector.axis.x + matrix.element.yy * vector.axis.y + matrix.element.yz * vector.axis.z,
        .z = matrix.element.zx * vector.axis.x + matrix.element.zy * vector.axis.y + matrix.element.zz * vector.axis.z,
    }};
}

/**
 * @brief ��Ԫ��ת��ת����
 */
static inline FusionMatrix FusionQuaternionToMatrix(const FusionQuaternion quaternion) {
    const float qwqw = quaternion.element.w * quaternion.element.w;
    const float qwqx = quaternion.element.w * quaternion.element.x;
    const float qwqy = quaternion.element.w * quaternion.element.y;
    const float qwqz = quaternion.element.w * quaternion.element.z;
    const float qxqy = quaternion.element.x * quaternion.element.y;
    const float qxqz = quaternion.element.x * quaternion.element.z;
    const float qyqz = quaternion.element.y * quaternion.element.z;
    return (FusionMatrix){.element = {
        .xx = 2.0f * (qwqw - 0.5f + quaternion.element.x * quaternion.element.x),
        .xy = 2.0f * (qxqy - qwqz),
        .xz = 2.0f * (qxqz + qwqy),
        .yx = 2.0f * (qxqy + qwqz),
        .yy = 2.0f * (qwqw - 0.5f + quaternion.element.y * quaternion.element.y),
        .yz = 2.0f * (qyqz - qwqx),
        .zx = 2.0f * (qxqz - qwqy),
        .zy = 2.0f * (qyqz + qwqx),
        .zz = 2.0f * (qwqw - 0.5f + quaternion.element.z * quaternion.element.z),
    }};
}

/**
 * @brief ��Ԫ��תŷ����
 */
static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion) {
    const float halfMinusQySquared = 0.5f - quaternion.element.y * quaternion.element.y;
    return (FusionEuler){.angle = {
        .roll = FusionRadiansToDegrees(atan2f(quaternion.element.w * quaternion.element.x + quaternion.element.y * quaternion.element.z, halfMinusQySquared - quaternion.element.x * quaternion.element.x)),
        .pitch = FusionRadiansToDegrees(FusionAsin(2.0f * (quaternion.element.w * quaternion.element.y - quaternion.element.z * quaternion.element.x))),
        .yaw = FusionRadiansToDegrees(atan2f(quaternion.element.w * quaternion.element.z + quaternion.element.x * quaternion.element.y, halfMinusQySquared - quaternion.element.z * quaternion.element.z)),
    }};
}

//------------------------------------------------------------------------------
// ��������ϵ����

typedef enum {
    FusionConventionNwu, /* ��-��-������ϵ */
    FusionConventionEnu, /* ��-��-������ϵ */
    FusionConventionNed, /* ��-��-������ϵ */
} FusionConvention;

//------------------------------------------------------------------------------
// AHRS�㷨�ṹ��

typedef struct {
    FusionConvention convention;  // ʹ�õ�����ϵԼ��
    float gain;                   // �㷨����
    float gyroscopeRange;         // ���������̷�Χ(��/s)
    float accelerationRejection;  // ���ٶȼƾܾ���ֵ
    unsigned int recoveryTriggerPeriod; // �ָ���������
    float restGain;              // gain when at rest for faster convergence
    float restGyroscopeThreshold;  // rest detection: max gyro magnitude (deg/s), default 5.0
    float restAccelerationMin;     // rest detection: min acc magnitude (g), default 0.9
    float restAccelerationMax;     // rest detection: max acc magnitude (g), default 1.1
    unsigned int restDebouncePeriod; // rest detection: debounce samples, default 50
    float dynamicFeedbackScale;    // dynamic feedback: scaling factor, default 50.0
} FusionAhrsSettings;

typedef struct {
    FusionAhrsSettings settings;  // �㷨����
    FusionQuaternion quaternion;  // ��ǰ��̬��Ԫ��
    FusionVector accelerometer;   // �洢�ļ��ٶȼ�����
		bool initialising;
    float rampedGain;
    float rampedGainStep;
    bool angularRateRecovery;     // �����ʻָ���־
    FusionVector halfAccelerometerFeedback; // ���ٶȼƷ�����(��ֵ)
    bool accelerometerIgnored;    // �Ƿ���Լ��ٶȼ�����
    int accelerationRecoveryTrigger; // ���ٶȻָ�������
    int accelerationRecoveryTimeout; // ���ٶȻָ���ʱ
    int restCounter;             // consecutive rest sample counter
    bool atRest;                 // rest state flag
} FusionAhrs;

typedef struct {
    float accelerationError;      // ���ٶ����(��)
    bool accelerometerIgnored;    // ���ٶȼ��Ƿ񱻺���
    float accelerationRecoveryTrigger; // ���ٶȻָ���������
} FusionAhrsInternalStates;

typedef struct {
		bool initialising;
    bool angularRateRecovery;     // �Ƿ��ڽ����ʻָ�״̬
    bool accelerationRecovery;    // �Ƿ��ڼ��ٶȻָ�״̬
    bool atRest;                  // true = static, false = dynamic
} FusionAhrsFlags;

// AHRS�㷨��������
void FusionAhrsInitialise(FusionAhrs *const ahrs);
void FusionAhrsReset(FusionAhrs *const ahrs);
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings);
void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs);
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion);
FusionVector FusionAhrsGetGravity(const FusionAhrs *const ahrs);
FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs);
FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs);
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs);
FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs);
void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading);

//------------------------------------------------------------------------------
// ��������ƫУ���㷨

typedef struct {
    float filterCoefficient;      // �˲���ϵ��
    unsigned int timeout;         // ��ʱ����
    unsigned int timer;           // ��ʱ��
    FusionVector gyroscopeOffset; // ��������ƫ����ֵ
} FusionOffset;

void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate);
FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);

//------------------------------------------------------------------------------
// �ݶ��½��㷨

#define sampleFreqDef   500.0f          // Ĭ�ϲ���Ƶ�� (Hz)
#define betaDef         0.1f            // Ĭ���㷨���� (2 * Kp)

typedef struct {
	float beta = betaDef;                // ��ʼ���㷨����
	float q0 = 1.0f;                     // ��ʼ����Ԫ������ʾ��ʼ��̬Ϊˮƽ
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;
	float invSampleFreq = 1.0f / sampleFreqDef; // ��ʼ������Ƶ�ʵĵ���
	float Fusion_Power = 0.4 ; //�ݶȷ���Fusion����ںϵ�Ȩ�� 
	float maxFusionPower = 0.5f; // ???????????
    float minFusionPower = 0.005f; // ???????????
	
	 
    float fq0, fq1, fq2, fq3; // ????��??????????????????
    float filterAlpha = 0.05; // ????????? (alpha), ????1?????��?????????????
	
	
    float v_s0 = 0 , v_s1 = 0 , v_s2 = 0, v_s3 = 0 ; // ?????????????
    float momentum = 0.9; // ??????? gamma
	
}  MadgwickParam ;

void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az) ;

#ifdef __cplusplus
}
#endif

#endif // FUSION_H