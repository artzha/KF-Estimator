/*
 * attitude.c
 *
 *  Created on: Dec 29, 2020
 *      Author: arthur
 */
#include <math.h>
#include "attitude.h"

#define betaDef         0.1 // 2 * proportional gain

void attitudeInit(AttitudeState *s) {
    // gyroscope drift estimated to be 1 deg/s
    s->beta = sqrt(3.0/4) * 3.14159265358979 * (1.0/180.0);
    s->q0 = 1.0;
    s->q1 = s->q2 = s->q3 = 0.0;
    s->step = 0.000018;  // step for gradient descent found through testing
}

void computeAngles(AttitudeState *s)
{
    float q0 = s->q0;
    float q1 = s->q1;
    float q2 = s->q2;
    float q3 = s->q3;
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
    float invNorm;
    float deltat = s->step;
    float beta = s->beta;
    float q_1, q_2, q_3, q_4;
    float qDot_omega_1, qDot_omega_2, qDot_omega_3, qDot_omega_4;  // quaternion derivative from gyroscopes elements
    float f_1, f_2, f_3;                                        // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;   // objective function Jacobian elements
    float qHatDot_1, qHatDot_2, qHatDot_3, qHatDot_4;   // estimated direction of the gyroscope error
    // Auxiliary variables to avoid repeated calculations
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

    // Normalize the accelerometer measurement
    invNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x *= invNorm;
    a_y *= invNorm;
    a_z *= invNorm;
    // Compute the objective function and Jacobian
    f_1 = twoq_2 * q_4 - twoq_1 * q_3 - a_x;
    f_2 = twoq_1 * q_2 + twoq_3 * q_4 - a_y;
    f_3 = 1.0 - twoq_2 * q_2 - twoq_3 * q_3 - a_z;
    J_11or24 = twoq_3;  // J_11 negated in matrix multiplication
    J_12or23 = 2.0 * q_4;
    J_13or22 = twoq_1;  // J_12 negated in matrix multiplication
    J_14or21 = twoq_2;
    J_32 = 2.0 * J_14or21;  // negated in matrix multiplication
    J_33 = 2.0 * J_11or24;  // negated in matrix multiplication

    // Compute the gradient (matrix multiplication)
    qHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    qHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    qHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    qHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    // Normalize the gradient
    invNorm = invSqrt(qHatDot_1 * qHatDot_1 + qHatDot_2 * qHatDot_2 + qHatDot_3 * qHatDot_3 + qHatDot_4 * qHatDot_4);
    qHatDot_1 *= invNorm;
    qHatDot_2 *= invNorm;
    qHatDot_3 *= invNorm;
    qHatDot_4 *= invNorm;

    // Compute the quaternion derivative measured by gyroscopes
    qDot_omega_1 = -halfq_2 * w_x - halfq_3 * w_y - halfq_4 * w_z;
    qDot_omega_2 = halfq_1 * w_x + halfq_3 * w_z - halfq_4 * w_y;
    qDot_omega_3 = halfq_1 * w_y - halfq_2 * w_z + halfq_4 * w_x;
    qDot_omega_4 = halfq_1 * w_z + halfq_2 * w_y - halfq_3 * w_x;

    // Compute then integrate the estimated quaternion derivative
    q_1 += (qDot_omega_1 - (beta * qHatDot_1)) * deltat;
    q_2 += (qDot_omega_2 - (beta * qHatDot_2)) * deltat;
    q_3 += (qDot_omega_3 - (beta * qHatDot_3)) * deltat;
    q_4 += (qDot_omega_4 - (beta * qHatDot_4)) * deltat;

    // Normalize quaternion
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

