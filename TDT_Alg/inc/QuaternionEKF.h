/**
 * @file QuaternionEKF.h
 * @brief Quaternion Extended Kalman Filter for attitude estimation
 * @note  State vector: [q0, q1, q2, q3, bx, by] (quaternion + gyro xy bias)
 *        Measurement:  normalized accelerometer [ax, ay, az]
 *        Reference: "四元数EKF姿态更新算法" by 韭菜的菜 (SUSTech)
 */

#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#ifdef __cplusplus

#include "Matrix.h"
#include <math.h>

class QuaternionEKF {
public:
    QuaternionEKF() = default;

    /**
     * @brief  Initialize the EKF
     * @param  q1  Quaternion process noise
     * @param  q2  Gyro bias process noise
     * @param  r   Accelerometer measurement noise
     * @param  lambda  Fading factor for gyro bias covariance (0 < lambda <= 1)
     * @param  chiSquareThreshold  Chi-square test threshold for outlier rejection
     */
    void init(float q1, float q2, float r, float lambda, float chiSquareThreshold);

    /**
     * @brief  Run one EKF update step
     * @param  gx, gy, gz  Gyroscope readings in rad/s
     * @param  ax, ay, az  Accelerometer readings (any unit, will be normalized)
     * @param  dt  Time step in seconds
     */
    void update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt);

    // Output angles (degrees)
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;

    // Estimated quaternion
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // Estimated gyro bias (rad/s)
    float gyroBias[3] = {0.0f, 0.0f, 0.0f};

    // Chi-square test value (for debug)
    float chiSquareVal = 0.0f;

    // Whether the filter has converged (chi-square dropped below threshold once)
    uint8_t converged = 0;

    // Whether accelerometer measurement was used in the last update (1=used, 0=skipped)
    uint8_t measUpdated = 0;

private:
    // EKF parameters
    float Q1_ = 10.0f;      // quaternion process noise
    float Q2_ = 0.001f;     // gyro bias process noise
    float R_  = 1000000.0f; // accel measurement noise
    float lambda_ = 0.9996f;
    float chiThreshold_ = 0.01f;

    // State vector x = [q0 q1 q2 q3 bx by]^T
    Matrixf<6, 1> xhat_;

    // Covariance
    Matrixf<6, 6> P_;

    bool initialized_ = false;

    // Rest detection counter: must be still for N frames before accel is trusted
    // At 500Hz, 250 frames = 0.5 seconds of stillness required
    int restCounter_ = 0;
    static const int REST_REQUIRED = 250;

    // Helper: fast inverse sqrt
    static float invSqrt(float x);
};

#endif // __cplusplus
#endif // QUATERNION_EKF_H
