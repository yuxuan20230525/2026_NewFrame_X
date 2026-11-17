/**
 * ahrs_gcc.c — GCC 版 AHRS 库实现
 * 替代 Keil 预编译的 AHRS.lib
 * 实现 Mahony 互补滤波 AHRS 算法
 */
#include "AHRS_middleware.h"
#include "ahrs_lib.h"
#include <math.h>

/* Mahony 滤波器增益 */
#define Kp 2.0f
#define Ki 0.01f

/* 积分误差 */
static fp32 integralFBx = 0.0f;
static fp32 integralFBy = 0.0f;
static fp32 integralFBz = 0.0f;

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    (void)mag;

    fp32 ax = accel[0], ay = accel[1], az = accel[2];
    fp32 norm = AHRS_invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    fp32 pitch = AHRS_asinf(-ax);
    fp32 roll  = AHRS_atan2f(ay, az);

    /* Euler → Quaternion (yaw = 0) */
    fp32 cy = 1.0f;  /* cos(0/2) */
    fp32 sy = 0.0f;  /* sin(0/2) */
    fp32 cp = AHRS_cosf(pitch * 0.5f);
    fp32 sp = AHRS_sinf(pitch * 0.5f);
    fp32 cr = AHRS_cosf(roll * 0.5f);
    fp32 sr = AHRS_sinf(roll * 0.5f);

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3])
{
    (void)mag;

    fp32 q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    fp32 gx = gyro[0], gy = gyro[1], gz = gyro[2];
    fp32 ax = accel[0], ay = accel[1], az = accel[2];
    fp32 halfT = timing_time * 0.5f;

    /* 归一化加速度 */
    fp32 recipNorm = ax * ax + ay * ay + az * az;
    if (recipNorm > 0.0f)
    {
        recipNorm = AHRS_invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* 估计重力方向 */
        fp32 vx = 2.0f * (q1 * q3 - q0 * q2);
        fp32 vy = 2.0f * (q0 * q1 + q2 * q3);
        fp32 vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* 误差 = 叉积 */
        fp32 ex = (ay * vz - az * vy);
        fp32 ey = (az * vx - ax * vz);
        fp32 ez = (ax * vy - ay * vx);

        /* 积分 */
        integralFBx += Ki * ex * timing_time;
        integralFBy += Ki * ey * timing_time;
        integralFBz += Ki * ez * timing_time;

        gx += Kp * ex + integralFBx;
        gy += Kp * ey + integralFBy;
        gz += Kp * ez + integralFBz;
    }

    /* 四元数微分方程 */
    fp32 dq0 = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    fp32 dq1 = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    fp32 dq2 = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    fp32 dq3 = ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    /* 归一化四元数 */
    recipNorm = AHRS_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    quat[0] = q0 * recipNorm;
    quat[1] = q1 * recipNorm;
    quat[2] = q2 * recipNorm;
    quat[3] = q3 * recipNorm;

    return 1;
}

void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw   = AHRS_atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                          1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
    *pitch = AHRS_asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));
    *roll  = AHRS_atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                          1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
}

fp32 get_yaw(const fp32 quat[4])
{
    return AHRS_atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                       1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
}

fp32 get_pitch(const fp32 quat[4])
{
    return AHRS_asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));
}

fp32 get_roll(const fp32 quat[4])
{
    return AHRS_atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                       1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
}

fp32 get_carrier_gravity(void)
{
    return 9.81f;
}
