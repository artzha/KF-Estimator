/*
 * attitude.h
 *
 *  Created on: Dec 29, 2020
 *      Author: arthur
 */

#ifndef INC_ATTITUDE_H_
#define INC_ATTITUDE_H_
#define ARM_MATH_CM4

#include "arm_math.h"
#include <stdint.h>
#include <stdlib.h>

#define Q_COVAR         0.0001
#define R_COVAR         0.00001    // estimate of std dev for position meas
#define A_MAT_SZ        4
#define X_MAT_SZ        6
#define G_MAT_SZ        2
#define H_MAT_SZ        2
#define Z_MAT_SZ        3
#define W_MAT_SZ        6
#define V_MAT_SZ        3
#define P_MAT_SZ        4
#define Q_MAT_SZ        4
#define R_MAT_SZ        1
#define I_MAT_SZ        4

typedef struct AttitudeState {
    float beta, step, filterBeta;
    float q0, q1, q2, q3;
    double roll, pitch, yaw;
    float invSampleFreq;
} AttitudeState;

typedef struct KalmanState {
    float32_t time;
    float32_t pos_var[3];
    float32_t acc_var[3];
    float32_t acc_bias[3];
    float32_t x_mat[X_MAT_SZ]; // 3 variables for each axis
    float32_t a_mat[A_MAT_SZ];
    float32_t g_mat[G_MAT_SZ];
    float32_t h_mat[H_MAT_SZ];
    float32_t z_mat[Z_MAT_SZ]; //  stores i j k pos
    float32_t w_mat[W_MAT_SZ];
    float32_t v_mat[V_MAT_SZ];
    float32_t p_mat[P_MAT_SZ];
    float32_t q_mat[Q_MAT_SZ];
    float32_t r_mat[R_MAT_SZ];
    float32_t i_mat[I_MAT_SZ];
    arm_matrix_instance_f32 X;  // Process State
    arm_matrix_instance_f32 A;  // Model Matrix
    arm_matrix_instance_f32 G;  // Random Disturbance
    arm_matrix_instance_f32 H;  // Observation Matrix (output)
    arm_matrix_instance_f32 Z;  // Observation Output
    arm_matrix_instance_f32 W;  // Process Noise
    arm_matrix_instance_f32 V;  // Observation Noise
    arm_matrix_instance_f32 P;  // Estimate Covariance
    arm_matrix_instance_f32 Q;  // Process Noise Covariance
    arm_matrix_instance_f32 R;  // Innovation Covariance Noise
    arm_matrix_instance_f32 I;  // Identity Matrix

    /* Variables for removing velocity drift rate */
    uint32_t num_samples_ns; // num of samples during non stationary
    uint8_t is_stationary; // boolean var if sensor was stationary
    float32_t prev_vel; // velocity at start of non stationary period
    float32_t drift_rate; // number of samples during non stationary period
} KalmanState;

void initKFMatrices(KalmanState* kf);

void attitudeInit(AttitudeState *s, KalmanState* kf);

void kfUpdate(KalmanState *kf, float* acc);

void normalizeGravity(AttitudeState *s, float* acc);

void madgwickUpdate(AttitudeState *s, float* acc, float* gyro, uint8_t sz);

void computeAngles(AttitudeState *s);

float32_t box_muller(float32_t m, float32_t s);

float invSqrt(float x);

#endif /* INC_ATTITUDE_H_ */
