/*
 * attitude.c
 *
 *  Created on: Dec 29, 2020
 *      Author: arthur
 */
#include "attitude.h"

#define betaDef         0.1 // 2 * proportional gain

void attitudeInit(AttitudeState *s, KalmanState* kf) {
    // gyroscope drift estimated to be 1 deg/s
    s->beta = sqrt(3.0/4) * 3.14159265358979 * (1.0/180.0);
    s->q0 = 1.0;
    s->q1 = s->q2 = s->q3 = 0.0;
    s->step = 0.0000175;  // step for gradient descent found through testing
    initKFMatrices(kf);
}

void initKFMatrices(KalmanState* kf) {
    /*
    * Kalman Filter array is 3 rows
    *  Each row represents an axis
    *  Each column represents x1, x2, x3 for their respective axis
    */
    kf->time = 0.001; // sample rate (s)
    kf->acc_var[0] = 0.000006004;
    kf->acc_var[1] = 0.000004081;
    kf->acc_var[2] = 0.000013743;
    kf->acc_bias[0]= -0.00432859;
    kf->acc_bias[1]= 0.000365862;
    kf->acc_bias[2]= 0.00188605;
    float32_t t = kf->time;
    float32_t t2= t*t;
    float32_t t2d2      = t2/2.0; //precompute
    float32_t t4d4      = t2d2*t2d2;
    float32_t t3d2      = t2d2*t;

    // Stores state of current axis
    for (uint8_t i = 0; i < X_MAT_SZ; ++i) {
        kf->x_mat[i] = 0;
    }
    arm_mat_init_f32(&kf->X, 3, 2, kf->x_mat);

    // A Matrix
    kf->a_mat[0] = 1;
    kf->a_mat[1] = t;
    kf->a_mat[2] = 0;
    kf->a_mat[3] = 1;
    arm_mat_init_f32(&kf->A, 2, 2, kf->a_mat);

    // G Matrix
    kf->g_mat[0] = t2d2;
    kf->g_mat[1] = t;
    arm_mat_init_f32(&kf->G, G_MAT_SZ, 1, kf->g_mat);

    // H Matrix
    kf->h_mat[0] = 1;
    kf->h_mat[1] = 0;
    arm_mat_init_f32(&kf->H, 1, H_MAT_SZ, kf->h_mat);

    // Z Matrix pos of current axis
    for (uint8_t i = 0; i < Z_MAT_SZ; ++i) {
        kf->z_mat[i] = 0;
    }
    arm_mat_init_f32(&kf->Z, 1, 1, kf->z_mat);

    // W Matrix
    for (uint8_t i = 0; i < W_MAT_SZ; ++i) {
        kf->w_mat[i] = 0;
    }
    kf->w_mat[0] = t2d2;
    kf->w_mat[1] = t;
    kf->w_mat[2] = t2d2;
    kf->w_mat[3] = t;
    kf->w_mat[4] = t2d2;
    kf->w_mat[5] = t;
    arm_mat_init_f32(&kf->W, 2, 1, kf->w_mat);

    // V Matrix measured accelerometer various for current axis
    for (uint8_t i = 0; i < V_MAT_SZ; ++i) kf->v_mat[i] = 0;
//    kf->v_mat[0] = t2d2; // TODO replace if needed
//    kf->v_mat[1] = t2d2;
//    kf->v_mat[2] = t2d2;
    arm_mat_init_f32(&kf->V, 1, 1, kf->v_mat);

    // P Matrix initially position is known to be true
    for(uint8_t i = 0; i < P_MAT_SZ; ++i) {
        kf->p_mat[i] = 0;
    }
    arm_mat_init_f32(&kf->P, 2, 2, kf->p_mat);

    // Q Matrix constant
    for (uint8_t i = 0; i < Q_MAT_SZ; ++i) {
        kf->q_mat[i] = 0;
    }
    kf->q_mat[0] = t4d4;
    kf->q_mat[1] = t3d2;
    kf->q_mat[2] = t3d2;
    kf->q_mat[3] = t2;
    arm_mat_init_f32(&kf->Q, 2, 2, kf->q_mat);
    arm_mat_scale_f32(&kf->Q, Q_COVAR, &kf->Q);

    // R Matrix
    kf->r_mat[0] = R_COVAR;
    arm_mat_init_f32(&kf->R, 1, 1, kf->r_mat);

    // Identity Matrix
    for (uint8_t i = 0; i < I_MAT_SZ; ++i) {
        kf->i_mat[i] = 0;
    }
    kf->i_mat[0] = 1;
    kf->i_mat[3] = 1;
    arm_mat_init_f32(&kf->I, 2, 2, kf->i_mat);
}

/* code borrowed from ftp://ftp.taygeta.com/pub/c/boxmuller.c */
float32_t box_muller(float32_t m, float32_t s)   /* normal random variate generator */
{
    /* mean m, standard deviation s */
    float32_t x1, x2, w, _y1;
    static float32_t y2;
    static int use_last = 0;

    if (use_last)               /* use value from previous call */
    {
        _y1 = y2;
        use_last = 0;
    }
    else
    {
        do {
            x1 = 2.0 * drand48() - 1.0;
            x2 = 2.0 * drand48() - 1.0;
            w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );

        w = sqrt( (-2.0 * log( w ) ) / w );
        _y1 = x1 * w;
        y2 = x2 * w;
        use_last = 1;
    }

    float32_t result = ( m + _y1 * s );

    return result;
}

void kfUpdate(KalmanState *kf, float* acc) {
    // Update Equations
    float32_t auxA_2_2_mat[4] = {0};
    float32_t auxB_2_2_mat[4] = {0};
    float32_t auxC_2_2_mat[4] = {0};
    float32_t auxP_2_2_mat[4] = {0};
    float32_t auxA_2_1_mat[2] = {0};
    float32_t auxB_2_1_mat[2] = {0};
    float32_t auxC_2_1_mat[2] = {0};
    float32_t auxD_2_1_mat[2] = {0};
    float32_t auxX_2_1_mat[2] = {0};
    float32_t auxK_2_1_mat[2] = {0};
    float32_t auxW_2_1_mat[2] = {0};
    float32_t auxA_1_1_mat[1] = {0};
    float32_t auxB_1_1_mat[1] = {0};
    float32_t auxC_1_1_mat[1] = {0};
    arm_matrix_instance_f32 auxA_2_2, auxB_2_2, auxC_2_2;
    arm_matrix_instance_f32 auxA_2_1, auxB_2_1, auxC_2_1, auxD_2_1;
    arm_matrix_instance_f32 auxA_1_1, auxB_1_1, auxC_1_1;
    arm_matrix_instance_f32 auxX_2_1, auxP_2_2;
    arm_matrix_instance_f32 auxK_2_1;
    arm_matrix_instance_f32 auxW_2_1;
    arm_mat_init_f32(&auxA_2_2, 2, 2, auxA_2_2_mat);
    arm_mat_init_f32(&auxB_2_2, 2, 2, auxB_2_2_mat);
    arm_mat_init_f32(&auxC_2_2, 2, 2, auxC_2_2_mat);
    arm_mat_init_f32(&auxP_2_2, 2, 2, auxP_2_2_mat);
    arm_mat_init_f32(&auxA_2_1, 2, 1, auxA_2_1_mat);
    arm_mat_init_f32(&auxB_2_1, 2, 1, auxB_2_1_mat);
    arm_mat_init_f32(&auxC_2_1, 2, 1, auxC_2_1_mat);
    arm_mat_init_f32(&auxD_2_1, 2, 1, auxD_2_1_mat);
    arm_mat_init_f32(&auxX_2_1, 2, 1, auxX_2_1_mat);
    arm_mat_init_f32(&auxK_2_1, 2, 1, auxK_2_1_mat);
    arm_mat_init_f32(&auxW_2_1, 2, 1, auxW_2_1_mat);
    arm_mat_init_f32(&auxA_1_1, 1, 1, auxA_1_1_mat);
    arm_mat_init_f32(&auxB_1_1, 1, 1, auxB_1_1_mat);
    arm_mat_init_f32(&auxC_1_1, 1, 1, auxC_1_1_mat);
    for (uint8_t i = 0; i < 3; ++i) {
        uint8_t X_mat_offset = i*2;
        // Predict Equations

        /* Set current state to respective coordinate axis */
        arm_mat_init_f32(&kf->X, 2, 1, kf->x_mat+X_mat_offset);
        arm_mat_init_f32(&kf->V, 1, 1, kf->v_mat+i);

        float32_t W_var = box_muller(0, kf->acc_var[i]);
        arm_mat_scale_f32(&kf->G, W_var, &auxW_2_1);

        /* G*ak: Update measured acceleration disturbance matrix */
        // conversion for gs to m/s^2
        arm_mat_scale_f32(&kf->G, acc[i]*9.80665, &auxA_2_1); // G*ak 3x1

        /* A*x_k-1k-1: Update process matrix */
        arm_mat_mult_f32(&kf->A, &kf->X, &auxB_2_1); // A*X 3x1

        /* sigma*W_k: Update position measurement noise matrix */
        arm_mat_scale_f32(&auxW_2_1, 1, &auxC_2_1); // W*1 3x1

        /* x_kk-1= A*x_k-1k-1 + G*ak + W */
        arm_mat_add_f32(&auxB_2_1, &auxA_2_1, &auxD_2_1); // A*X + G*ak
        arm_mat_add_f32(&auxD_2_1, &auxC_2_1, &auxX_2_1); //X = A*X + G*ak + W*sigma

        /*P_kk-1= APA_T + Qk */
        arm_mat_trans_f32(&kf->A, &auxA_2_2);       // A_t 3x3
        arm_mat_mult_f32(&kf->P, &auxA_2_2, &auxB_2_2); // P*A_t
        arm_mat_mult_f32(&kf->A, &auxB_2_2, &auxC_2_2); // A*P*A_t
        arm_mat_add_f32(&auxC_2_2, &kf->Q, &auxP_2_2);  // P = A*P*A_t + Q 3x3

        // Update Equations

        /* Kalman gain: K = P_kk-1*Ht_k*S^-1_k */
        arm_mat_trans_f32(&kf->H, &auxA_2_1);   // H_t
        arm_mat_mult_f32(&auxP_2_2, &auxA_2_1, &auxB_2_1);   // PH_t 3x1

        arm_mat_mult_f32(&kf->H, &auxB_2_1, &auxA_1_1); // HPH_t 1x1
        arm_mat_add_f32(&auxA_1_1, &kf->R, &auxB_1_1); // S = HPH_t + R 1x1
        arm_mat_inverse_f32(&auxB_1_1, &auxC_1_1); // S^-1

        arm_mat_mult_f32(&auxA_2_1, &auxC_1_1, &auxC_2_1); // H_t * S^-1 3x1
        arm_mat_mult_f32(&auxP_2_2, &auxC_2_1, &auxK_2_1); // K = P * H^t * S^-1

        /* A posteriori state estimate */
        arm_mat_mult_f32(&auxK_2_1, &kf->H, &auxA_2_2); // KH
        arm_mat_sub_f32(&kf->I, &auxA_2_2, &auxB_2_2); // I-KH
        arm_mat_mult_f32(&auxB_2_2, &auxX_2_1, &auxA_2_1); // (I-KH)*X

        arm_mat_mult_f32(&auxA_2_2, &auxX_2_1, &auxB_2_1); // KHX
        arm_mat_mult_f32(&auxK_2_1, &kf->V, &auxC_2_1); // Kv
        arm_mat_add_f32(&auxA_2_1, &auxB_2_1, &auxD_2_1); // (I-KH)*X + KHX
        arm_mat_add_f32(&auxD_2_1, &auxC_2_1, &kf->X); // (I-KH)*X + KHX + Kv

        /* A posteriori estimate cov */
        arm_mat_mult_f32(&auxB_2_2, &auxP_2_2, &kf->P); // (I-KH)*P
    }
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

void normalizeGravity(AttitudeState *s, float* acc) {
    float q0, q1, q2, q3;
    float q0q0, q1q1, q2q2, q3q3;
    float q0q1, q0q2, q0q3;
    float q1q2, q1q3;
    float q2q3;
    q0 = s->q0;
    q1 = s->q1;
    q2 = s->q2;
    q3 = s->q3;
    q0q0 = q0*q0;
    q1q1 = q1*q1;
    q2q2 = q2*q2;
    q3q3 = q3*q3;
    q0q1 = q0*q1;
    q0q2 = q0*q2;
    q0q3 = q0*q3;
    q1q2 = q1*q2;
    q1q3 = q1*q3;
    q2q3 = q2*q3;

    // rotate acceleration vector by quaternion orientation
    // essentially rotating sensor frame into inertial frame
    float r_acc[3];
    r_acc[0] = q0q0*acc[0] + 2*q0q2*acc[2] - 2*q0q3*acc[1] +
                    q1q1*acc[0] + 2*q1q2*acc[1] + 2*q1q3*acc[2] -
                    q3q3*acc[0] - q2q2*acc[0];
    r_acc[1] = 2*q1q2*acc[0] + q2q2*acc[1] + 2*q2q3*acc[2] +
                2*q0q3*acc[0] - q3q3*acc[1] + q0q0*acc[1] -
                2*q0q1*acc[2] - q1q1*acc[1];
    r_acc[2] = 2*q1q3*acc[0] + 2*q2q3*acc[1] + q3q3*acc[2] -
                2*q0q2*acc[0] - q2q2*acc[2] + 2*q0q1*acc[1] -
                q1q1*acc[2] + q0q0*acc[2];
    acc[0] = r_acc[0];
    acc[1] = r_acc[1];
    acc[2] = r_acc[2] - 1; // subtract out gravity component from acc
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

