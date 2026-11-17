/**
 * @file QuaternionEKF.cpp
 * @brief Quaternion Extended Kalman Filter for attitude estimation
 * @note  Implements the algorithm from "四元数EKF姿态更新算法"
 *        State: x = [q0, q1, q2, q3, bx, by]^T
 *        Measurement: z = normalized_accel [ax, ay, az]^T
 *        Features: chi-square test, fading factor, M-matrix yaw protection
 */

#include "QuaternionEKF.h"
#include <string.h>

float QuaternionEKF::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void QuaternionEKF::init(float q1, float q2, float r, float lambda, float chiSquareThreshold) {
    Q1_ = q1;
    Q2_ = q2;
    R_  = r;
    lambda_ = (lambda > 1.0f) ? 1.0f : lambda;
    chiThreshold_ = chiSquareThreshold;

    // Initial state: unit quaternion, zero bias
    xhat_ = Matrixf<6, 1>::zeros();
    xhat_[0][0] = 1.0f;

    // Initial covariance: high uncertainty for quaternion, moderate for bias
    P_ = Matrixf<6, 6>::zeros();
    P_[0][0] = 100000.0f;
    P_[1][1] = 100000.0f;
    P_[2][2] = 100000.0f;
    P_[3][3] = 100000.0f;
    P_[4][4] = 10000.0f;
    P_[5][5] = 10000.0f;
    // Small off-diagonal coupling
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            if (i != j) P_[i][j] = 0.1f;

    converged = 0;
    initialized_ = true;

    for (int i = 0; i < 4; i++) q[i] = xhat_[i][0];
    gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;
}

void QuaternionEKF::update(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt) {
    if (!initialized_) return;

    // Bias-compensated gyro
    float omega_x = gx - xhat_[4][0];
    float omega_y = gy - xhat_[5][0];
    float omega_z = gz; // z-axis bias not estimated

    float halfdt = 0.5f * dt;

    // ==================== Prediction ====================

    // State prediction: x'(k) = f(x(k-1), u)
    float q0 = xhat_[0][0], q1 = xhat_[1][0], q2 = xhat_[2][0], q3 = xhat_[3][0];

    // Quaternion kinematic equation (first-order Euler)
    Matrixf<6, 1> xpred = Matrixf<6, 1>::zeros();
    xpred[0][0] = q0 + halfdt * (-omega_x * q1 - omega_y * q2 - omega_z * q3);
    xpred[1][0] = q1 + halfdt * ( omega_x * q0 + omega_z * q2 - omega_y * q3);
    xpred[2][0] = q2 + halfdt * ( omega_y * q0 - omega_z * q1 + omega_x * q3);
    xpred[3][0] = q3 + halfdt * ( omega_z * q0 + omega_y * q1 - omega_x * q2);
    xpred[4][0] = xhat_[4][0]; // bias constant model
    xpred[5][0] = xhat_[5][0];

    // Normalize predicted quaternion
    float qnorm = invSqrt(xpred[0][0] * xpred[0][0] + xpred[1][0] * xpred[1][0] +
                           xpred[2][0] * xpred[2][0] + xpred[3][0] * xpred[3][0]);
    for (int i = 0; i < 4; i++) xpred[i][0] *= qnorm;

    // Jacobian F = df/dx
    float hgxdt = halfdt * omega_x;
    float hgydt = halfdt * omega_y;
    float hgzdt = halfdt * omega_z;

    Matrixf<6, 6> F = Matrixf<6, 6>::eye();
    // Row 0: dq0'/dx
    F[0][1] = -hgxdt;  F[0][2] = -hgydt;  F[0][3] = -hgzdt;
    F[0][4] =  q1 * halfdt;  F[0][5] =  q2 * halfdt;
    // Row 1: dq1'/dx
    F[1][0] =  hgxdt;  F[1][2] =  hgzdt;  F[1][3] = -hgydt;
    F[1][4] = -q0 * halfdt;  F[1][5] =  q3 * halfdt;
    // Row 2: dq2'/dx
    F[2][0] =  hgydt;  F[2][1] = -hgzdt;  F[2][3] =  hgxdt;
    F[2][4] = -q3 * halfdt;  F[2][5] = -q0 * halfdt;
    // Row 3: dq3'/dx
    F[3][0] =  hgzdt;  F[3][1] =  hgydt;  F[3][2] = -hgxdt;
    F[3][4] =  q2 * halfdt;  F[3][5] = -q1 * halfdt;
    // Row 4,5: bias unchanged (already identity)

    // Process noise Q
    Matrixf<6, 6> Q = Matrixf<6, 6>::zeros();
    Q[0][0] = Q1_ * dt;  Q[1][1] = Q1_ * dt;
    Q[2][2] = Q1_ * dt;  Q[3][3] = Q1_ * dt;
    Q[4][4] = Q2_ * dt;  Q[5][5] = Q2_ * dt;

    // Covariance prediction: P'(k) = F * P * F^T + Q
    Matrixf<6, 6> Ppred = F * P_ * F.trans() + Q;

    // ==================== Measurement Update ====================

    float accelNorm = ax * ax + ay * ay + az * az;

    // Rest detection: increment counter only when both gyro and accel indicate stillness
    bool isStill = (omega_x * omega_x + omega_y * omega_y + omega_z * omega_z < 0.01f)
                && (accelNorm > 0.81f && accelNorm < 1.21f);
    if (isStill) {
        if (restCounter_ < REST_REQUIRED) restCounter_++;
    } else {
        restCounter_ = 0;
    }

    // Only allow accel measurement update after being still for REST_REQUIRED frames
    if (restCounter_ < REST_REQUIRED) {
        measUpdated = 0;
        xhat_ = xpred;
        P_ = Ppred;
        goto output;
    }
    {
        float accelInvNorm = invSqrt(accelNorm);
        float nax = ax * accelInvNorm;
        float nay = ay * accelInvNorm;
        float naz = az * accelInvNorm;

        // Predicted quaternion elements
        float pq0 = xpred[0][0], pq1 = xpred[1][0];
        float pq2 = xpred[2][0], pq3 = xpred[3][0];

        // Measurement model h(x) = rotation of [0,0,1] by quaternion
        Matrixf<3, 1> hx = Matrixf<3, 1>::zeros();
        hx[0][0] = 2.0f * (pq1 * pq3 - pq0 * pq2);
        hx[1][0] = 2.0f * (pq0 * pq1 + pq2 * pq3);
        hx[2][0] = pq0 * pq0 - pq1 * pq1 - pq2 * pq2 + pq3 * pq3;

        // Jacobian H = dh/dx (3x6)
        float dq0 = 2.0f * pq0, dq1 = 2.0f * pq1;
        float dq2 = 2.0f * pq2, dq3 = 2.0f * pq3;

        Matrixf<3, 6> H = Matrixf<3, 6>::zeros();
        H[0][0] = -dq2;  H[0][1] =  dq3;  H[0][2] = -dq0;  H[0][3] =  dq1;
        H[1][0] =  dq1;  H[1][1] =  dq0;  H[1][2] =  dq3;  H[1][3] =  dq2;
        H[2][0] =  dq0;  H[2][1] = -dq1;  H[2][2] = -dq2;  H[2][3] =  dq3;
        // H[*][4] and H[*][5] remain 0 (bias doesn't affect measurement directly)

        // Measurement noise R
        Matrixf<3, 3> Rm = Matrixf<3, 3>::zeros();
        Rm[0][0] = R_;  Rm[1][1] = R_;  Rm[2][2] = R_;

        // Innovation S = H * P' * H^T + R
        Matrixf<3, 6> HT6 = H;
        Matrixf<6, 3> HT = H.trans();
        Matrixf<3, 3> S = H * Ppred * HT + Rm;

        // Innovation (residual)
        Matrixf<3, 1> z = Matrixf<3, 1>::zeros();
        z[0][0] = nax;  z[1][0] = nay;  z[2][0] = naz;
        Matrixf<3, 1> residual = z - hx;

        // Chi-square test: chi2 = r^T * S^-1 * r
        Matrixf<3, 3> Sinv = S.inv();
        Matrixf<1, 1> chi2mat = residual.trans() * Sinv * residual;
        chiSquareVal = chi2mat[0][0];

        // Check convergence
        if (chiSquareVal < 0.1f * chiThreshold_) {
            converged = 1;
        }

        if (chiSquareVal > chiThreshold_ && converged) {
            // Failed chi-square test: skip measurement update (prediction only)
            measUpdated = 0;
            xhat_ = xpred;
            P_ = Ppred;
        } else {
            measUpdated = 1;
            // Apply fading factor to gyro bias covariance (prevent over-convergence)
            Ppred[4][4] /= lambda_;
            Ppred[5][5] /= lambda_;

            // Kalman gain K = P' * H^T * S^-1
            Matrixf<6, 3> K = Ppred * HT * Sinv;

            // Correction vector
            Matrixf<6, 1> correction = K * residual;

            // M-matrix: zero out q3 correction to protect yaw angle
            // Accelerometer cannot observe yaw, so don't let it corrupt yaw
            correction[3][0] = 0.0f;

            // State update
            xhat_ = xpred + correction;

            // Covariance update: P = (I - K*H) * P'
            Matrixf<6, 6> I6 = Matrixf<6, 6>::eye();
            P_ = (I6 - K * H) * Ppred;
        }

        // Normalize updated quaternion
        float updNorm = invSqrt(xhat_[0][0] * xhat_[0][0] + xhat_[1][0] * xhat_[1][0] +
                                 xhat_[2][0] * xhat_[2][0] + xhat_[3][0] * xhat_[3][0]);
        for (int i = 0; i < 4; i++) xhat_[i][0] *= updNorm;
    }

output:
    // Export results
    q[0] = xhat_[0][0];
    q[1] = xhat_[1][0];
    q[2] = xhat_[2][0];
    q[3] = xhat_[3][0];
    gyroBias[0] = xhat_[4][0];
    gyroBias[1] = xhat_[5][0];
    gyroBias[2] = 0.0f;

    // Quaternion to Euler (ZYX convention, NWU frame)
    yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                   2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.295779513f;
    roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
}
