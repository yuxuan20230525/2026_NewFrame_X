/**
 * @file Fusion_AHRS.h
 * @author liuskywalkerjskd
 * @brief 基于加速度计和陀螺仪的AHRS算法实现
 * @note 移除了原版中所有磁力计相关代码，仅保留加速度计和陀螺仪部分
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
// 数学库定义

/**
 * @brief 3D向量结构体
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
 * @brief 四元数结构体
 */
typedef union {
    float array[4];
    struct {
        float w;  // 实部
        float x;  // i虚部
        float y;  // j虚部
        float z;  // k虚部
    } element;
} FusionQuaternion;

/**
 * @brief 3x3矩阵（按行主序存储）
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
 * @brief 欧拉角结构体
 * @note 滚转(roll)、俯仰(pitch)、偏航(yaw)分别对应绕X、Y、Z轴的旋转
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
 * @brief 零向量常量
 */
#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {0.0f, 0.0f, 0.0f} })

/**
 * @brief 单位向量常量
 */
#define FUSION_VECTOR_ONES ((FusionVector){ .array = {1.0f, 1.0f, 1.0f} })

/**
 * @brief 单位四元数常量
 */
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){ .array = {1.0f, 0.0f, 0.0f, 0.0f} })

/**
 * @brief 单位矩阵常量
 */
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){ .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}} })

/**
 * @brief 零欧拉角常量
 */
#define FUSION_EULER_ZERO ((FusionEuler){ .array = {0.0f, 0.0f, 0.0f} })

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

//------------------------------------------------------------------------------
// 数学库内联函数

/**
 * @brief 度转弧度
 */
static inline float FusionDegreesToRadians(const float degrees) {
    return degrees * ((float) M_PI / 180.0f);
}

/**
 * @brief 弧度转度
 */
static inline float FusionRadiansToDegrees(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

/**
 * @brief 安全的反正弦函数
 */
static inline float FusionAsin(const float value) {
    if (value <= -1.0f) return (float) M_PI / -2.0f;
    if (value >= 1.0f) return (float) M_PI / 2.0f;
    return asinf(value);
}

/**
 * @brief 快速反平方根计算（近似算法）
 */
static inline float FusionFastInverseSqrt(const float x) {
    typedef union { float f; int32_t i; } Union32;
    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

/**
 * @brief 判断向量是否为0
 */
static inline bool FusionVectorIsZero(const FusionVector vector) {
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

/**
 * @brief 向量加法
 */
static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x + vectorB.axis.x,
        .y = vectorA.axis.y + vectorB.axis.y,
        .z = vectorA.axis.z + vectorB.axis.z,
    }};
}

/**
 * @brief 向量减法
 */
static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x - vectorB.axis.x,
        .y = vectorA.axis.y - vectorB.axis.y,
        .z = vectorA.axis.z - vectorB.axis.z,
    }};
}

/**
 * @brief 向量各分量求和
 */
static inline float FusionVectorSum(const FusionVector vector) {
    return vector.axis.x + vector.axis.y + vector.axis.z;
}

/**
 * @brief 向量标量乘法
 */
static inline FusionVector FusionVectorMultiplyScalar(const FusionVector vector, const float scalar) {
    return (FusionVector){.axis = {
        .x = vector.axis.x * scalar,
        .y = vector.axis.y * scalar,
        .z = vector.axis.z * scalar,
    }};
}

/**
 * @brief 向量逐元素乘法（Hadamard积）
 */
static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x * vectorB.axis.x,
        .y = vectorA.axis.y * vectorB.axis.y,
        .z = vectorA.axis.z * vectorB.axis.z,
    }};
}

/**
 * @brief 向量叉积
 */
static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.y * vectorB.axis.z - vectorA.axis.z * vectorB.axis.y,
        .y = vectorA.axis.z * vectorB.axis.x - vectorA.axis.x * vectorB.axis.z,
        .z = vectorA.axis.x * vectorB.axis.y - vectorA.axis.y * vectorB.axis.x,
    }};
}

/**
 * @brief 向量点积
 */
static inline float FusionVectorDotProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return FusionVectorSum(FusionVectorHadamardProduct(vectorA, vectorB));
}

/**
 * @brief 向量模的平方
 */
static inline float FusionVectorMagnitudeSquared(const FusionVector vector) {
    return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

/**
 * @brief 向量模
 */
static inline float FusionVectorMagnitude(const FusionVector vector) {
    return sqrtf(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief 向量归一化
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
    const float magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
    return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}

/**
 * @brief 四元数加法
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
 * @brief 四元数乘法
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
 * @brief 四元数与向量乘法
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
 * @brief 四元数归一化
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
 * @brief 矩阵与向量乘法
 */
static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector) {
    return (FusionVector){.axis = {
        .x = matrix.element.xx * vector.axis.x + matrix.element.xy * vector.axis.y + matrix.element.xz * vector.axis.z,
        .y = matrix.element.yx * vector.axis.x + matrix.element.yy * vector.axis.y + matrix.element.yz * vector.axis.z,
        .z = matrix.element.zx * vector.axis.x + matrix.element.zy * vector.axis.y + matrix.element.zz * vector.axis.z,
    }};
}

/**
 * @brief 四元数转旋转矩阵
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
 * @brief 四元数转欧拉角
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
// 地球坐标系定义

typedef enum {
    FusionConventionNwu, /* 北-西-上坐标系 */
    FusionConventionEnu, /* 东-北-上坐标系 */
    FusionConventionNed, /* 北-东-下坐标系 */
} FusionConvention;

//------------------------------------------------------------------------------
// AHRS算法结构体

typedef struct {
    FusionConvention convention;  // 使用的坐标系约定
    float gain;                   // 算法增益
    float gyroscopeRange;         // 陀螺仪量程范围(°/s)
    float accelerationRejection;  // 加速度计拒绝阈值
    unsigned int recoveryTriggerPeriod; // 恢复触发周期
} FusionAhrsSettings;

typedef struct {
    FusionAhrsSettings settings;  // 算法设置
    FusionQuaternion quaternion;  // 当前姿态四元数
    FusionVector accelerometer;   // 存储的加速度计数据
		bool initialising;
    float rampedGain;
    float rampedGainStep;
    bool angularRateRecovery;     // 角速率恢复标志
    FusionVector halfAccelerometerFeedback; // 加速度计反馈量(半值)
    bool accelerometerIgnored;    // 是否忽略加速度计数据
    int accelerationRecoveryTrigger; // 加速度恢复触发器
    int accelerationRecoveryTimeout; // 加速度恢复超时
} FusionAhrs;

typedef struct {
    float accelerationError;      // 加速度误差(度)
    bool accelerometerIgnored;    // 加速度计是否被忽略
    float accelerationRecoveryTrigger; // 加速度恢复触发进度
} FusionAhrsInternalStates;

typedef struct {
		bool initialising;
    bool angularRateRecovery;     // 是否处于角速率恢复状态
    bool accelerationRecovery;    // 是否处于加速度恢复状态
} FusionAhrsFlags;

// AHRS算法函数声明
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
// 陀螺仪零偏校正算法

typedef struct {
    float filterCoefficient;      // 滤波器系数
    unsigned int timeout;         // 超时计数
    unsigned int timer;           // 计时器
    FusionVector gyroscopeOffset; // 陀螺仪零偏估计值
} FusionOffset;

void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate);
FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);

//------------------------------------------------------------------------------
// 梯度下降算法

#define sampleFreqDef   500.0f          // 默认采样频率 (Hz)
#define betaDef         0.1f            // 默认算法增益 (2 * Kp)

typedef struct {
	float beta = betaDef;                // 初始化算法增益
	float q0 = 1.0f;                     // 初始化四元数，表示初始姿态为水平
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;
	float invSampleFreq = 1.0f / sampleFreqDef; // 初始化采样频率的倒数
	float Fusion_Power = 0.4 ; //梯度法与Fusion结果融合的权重 
	float maxFusionPower = 0.5f; // ???????????
    float minFusionPower = 0.005f; // ???????????
	
	 
    float fq0, fq1, fq2, fq3; // ????洢??????????????????
    float filterAlpha = 0.05; // ????????? (alpha), ????1?????Ч?????????????
	
	
    float v_s0 = 0 , v_s1 = 0 , v_s2 = 0, v_s3 = 0 ; // ?????????????
    float momentum = 0.9; // ??????? gamma
	
}  MadgwickParam ;

void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az) ;

#ifdef __cplusplus
}
#endif

#endif // FUSION_H