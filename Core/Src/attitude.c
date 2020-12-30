/*
 * attitude.c
 *
 *  Created on: Dec 29, 2020
 *      Author: arthur
 */
#include <math.h>
#include "attitude.h"

#define betaDef         0.1 // 2 * proportional gain
#define sampleFreqDef   1000.0; // 100 Hz

void attitudeInit(AttitudeState *s) {
    s->filterBeta = sqrt(3.0/4) * 3.14159265358979 * (1.0/180.0);
    s->beta = betaDef;
    s->q0 = 1.0;
    s->q1 = s->q2 = s->q3 = 0.0;
    s->invSampleFreq = 1.0 / sampleFreqDef;
    s->step = 0.00002;  // step for gradient descent 0.00002
}

void attitudeUpdate(AttitudeState *s, float* acc, float* gyro, uint8_t sz) {
    float qG0, qG1, qG2, qG3;
    float qA0, qA1, qA2, qA3;
    float qDot0, qDot1, qDot2, qDot3;
    float fDot0, fDot1, fDot2, fDot3;
    float beta = s->beta;
    float step = s->step;
    float invSampleFreq = s->invSampleFreq;
    /* Convert gyroscope to rad/s from deg/s */
    float q0 = s->q0;
    float q1 = s->q1;
    float q2 = s->q2;
    float q3 = s->q3;
    float q0q0  = q0*q0;
    float q1q1  = q1*q1;
    float q2q2  = q2*q2;
    float q3q3  = q3*q3;
    float fDotNorm;
    float qANorm, qGNorm, qNorm;
    float wNorm, aNorm;
    // gyro is already in deg/s
    float wx = gyro[0];
    float wy = gyro[1];
    float wz = gyro[2];
    wNorm = invSqrt(wx*wx + wy*wy + wz*wz);
    wx *= wNorm;
    wy *= wNorm;
    wz *= wNorm;

    float ax = acc[0];
    float ay = acc[1];
    float az = acc[2];
    aNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= aNorm;
    ay *= aNorm;
    az *= aNorm;

    // Rotation change from gyroscope
    qDot0 = -wx*q1 - wy*q2 - wz*q3;
    qDot1 = wx*q0 + wz*q2 - wy*q3;
    qDot2 = wy*q0 - wz*q1 + wx*q3;
    qDot3 = wz*q0 + wy*q1 - wx*q2;

    // Integrate gyro rotation to compute gyro quaternion
    qG0 = q0 + qDot0*invSampleFreq;
    qG1 = q1 + qDot1*invSampleFreq;
    qG2 = q2 + qDot2*invSampleFreq;
    qG3 = q3 + qDot3*invSampleFreq;

    // Normalize gyro quaternion
//    qGNorm = invSqrt(qG0*qG0 + qG1*qG1 + qG2*qG2 + qG3*qG3);
//    qG0 *= qGNorm;
//    qG1 *= qGNorm;
//    qG2 *= qGNorm;
//    qG3 *= qGNorm;

    // Gradient descent step to minimize error f
    fDot0 = 8*q2q2*q0 + 8*q1q1*q0 + 4*ax*q2 - 4*ay*q1;
    fDot1 = 8*q3q3*q1 + 8*q0q0*q1 + 16*q2q2*q1 + 16*q3q3*q3 -
            4*ax*q3 - 4*ay*q0 + 8*az*q1 - 8*q1;
    fDot2 = 8*q0q0*q2 + 4*ax*q0 + 8*q3q3*q2 - 4*ay*q3 - 8*q2 +
            16*q1q1*q2 + 16*q2q2*q2 + 8*az*q2;
    fDot3 = 8*q1q1*q3 + 8*q2q2*q3 - 4*ax*q1 - 4*ay*q2;
    fDotNorm = invSqrt(fDot0*fDot0 + fDot1*fDot1
                            + fDot2*fDot2 + fDot3*fDot3);
    fDot0 *= fDotNorm;
    fDot1 *= fDotNorm;
    fDot2 *= fDotNorm;
    fDot3 *= fDotNorm;

    // Compute new acceleration quaternion
    qA0 = q0 - step * fDot0;
    qA1 = q1 - step * fDot1;
    qA2 = q2 - step * fDot2;
    qA3 = q3 - step * fDot3;

    // Normalize accel quaternion
    qANorm = invSqrt(qA0*qA0 + qA1*qA1 + qA2*qA2 + qA3*qA3);
    qA0 *= qANorm;
    qA1 *= qANorm;
    qA2 *= qANorm;
    qA3 *= qANorm;

    // Apply complementary filter formula
    q0 = beta*qG0 - (1-beta)*qA0;
    q1 = beta*qG1 - (1-beta)*qA1;
    q2 = beta*qG2 - (1-beta)*qA2;
    q3 = beta*qG3 - (1-beta)*qA3;

    // Normalize complementary quaternion in result
    qNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    s->q0 = q0 * qNorm;
    s->q1 = q1 * qNorm;
    s->q2 = q2 * qNorm;
    s->q3 = q3 * qNorm;
}

void computeAngles(AttitudeState *s)
{
    float q0 = s->q0;
    float q1 = s->q1;
    float q2 = s->q2;
    float q3 = s->q3;
//    s->roll = atan2f(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2) * 57.29578;
//    s->pitch = asinf(-2.0 * (q1*q3 - q0*q2)) * 57.29578;
//    s->yaw = atan2f(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3) * 57.29578;
    float q2q2 = q2 * q2;
    float t0 = -2.0 * (q2q2 + q3 * q3) + 1.0;
    float t1 = 2.0 * (q1 * q2 + q0 * q3);
    float t2 = -2.0 * (q1 * q3 - q0 * q2);
    float t3 = 2.0 * (q2 * q3 + q0 * q1);
    float t4 = -2.0 * (q1 * q1 + q2q2) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    s->pitch = asin(t2) * 57.29578;
    s->roll = atan2(t3, t4) * 57.29578;
    s->yaw = atan2(t1, t0) * 57.29578;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void madgwickUpdate(AttitudeState *s, float* acc, float* gyro, uint8_t sz) {
    float w_x, w_y, w_z, a_x, a_y, a_z;
    // vector norm
    float invNorm;
    float deltat = s->step;
    float beta = s->filterBeta;
    float q_1, q_2, q_3, q_4;
    float qDot_omega_1, qDot_omega_2, qDot_omega_3, qDot_omega_4;  // quaternion derivative from gyroscopes elements
    float f_1, f_2, f_3;                                        // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;   // objective function Jacobian elements
    float qHatDot_1, qHatDot_2, qHatDot_3, qHatDot_4;   // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    w_x = gyro[0];
    w_y = gyro[1];
    w_z = gyro[2];
    a_x = acc[0];
    a_y = acc[1];
    a_z = acc[2];
    q_1 = s->q0;
    q_2 = s->q1;
    q_3 = s->q2;
    q_4 = s->q3;
    float halfq_1 = 0.5 * q_1;
    float halfq_2 = 0.5 * q_2;
    float halfq_3 = 0.5 * q_3;
    float halfq_4 = 0.5 * q_4;
    float twoq_1 = 2.0 * q_1;
    float twoq_2 = 2.0 * q_2;
    float twoq_3 = 2.0 * q_3;

    // Normalise the accelerometer measurement
    invNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x *= invNorm;
    a_y *= invNorm;
    a_z *= invNorm;
    // Compute the objective function and Jacobian
    f_1 = twoq_2 * q_4 - twoq_1 * q_3 - a_x;
    f_2 = twoq_1 * q_2 + twoq_3 * q_4 - a_y;
    f_3 = 1.0 - twoq_2 * q_2 - twoq_3 * q_3 - a_z;
    J_11or24 = twoq_3;
    J_12or23 = 2.0 * q_4;
    J_13or22 = twoq_1;
    J_14or21 = twoq_2;
    J_32 = 2.0 * J_14or21;
    J_33 = 2.0 * J_11or24;
    // Compute the gradient (matrix multiplication)
    qHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    qHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    qHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    qHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // J_11 negated in matrix multiplication
    // J_12 negated in matrix multiplication
    // negated in matrix multiplication // negated in matrix multiplication
    // Normalise the gradient
    invNorm = invSqrt(qHatDot_1 * qHatDot_1 + qHatDot_2 * qHatDot_2 + qHatDot_3 * qHatDot_3 + qHatDot_4 * qHatDot_4);
    qHatDot_1 *= invNorm;
    qHatDot_2 *= invNorm;
    qHatDot_3 *= invNorm;
    qHatDot_4 *= invNorm;
    // Compute the quaternion derrivative measured by gyroscopes
    qDot_omega_1 = -halfq_2 * w_x - halfq_3 * w_y - halfq_4 * w_z;
    qDot_omega_2 = halfq_1 * w_x + halfq_3 * w_z - halfq_4 * w_y;
    qDot_omega_3 = halfq_1 * w_y - halfq_2 * w_z + halfq_4 * w_x;
    qDot_omega_4 = halfq_1 * w_z + halfq_2 * w_y - halfq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    q_1 += (qDot_omega_1 - (beta * qHatDot_1)) * deltat;
    q_2 += (qDot_omega_2 - (beta * qHatDot_2)) * deltat;
    q_3 += (qDot_omega_3 - (beta * qHatDot_3)) * deltat;
    q_4 += (qDot_omega_4 - (beta * qHatDot_4)) * deltat;
    // Normalise quaternion
    invNorm = invSqrt(q_1 * q_1 + q_2 * q_2 + q_3 * q_3 + q_4 * q_4);
    q_1 *= invNorm;
    q_2 *= invNorm;
    q_3 *= invNorm;
    q_4 *= invNorm;
    // Save new quaternion to state
    s->q0 = q_1;
    s->q1 = q_2;
    s->q2 = q_3;
    s->q3 = q_4;
}

