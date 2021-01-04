/*
 * attitude.h
 *
 *  Created on: Dec 29, 2020
 *      Author: arthur
 */

#ifndef INC_ATTITUDE_H_
#define INC_ATTITUDE_H_

#include <stdint.h>

typedef struct AttitudeState {
    float beta, step, filterBeta;
    float q0, q1, q2, q3;
    double roll, pitch, yaw;
    float invSampleFreq;
} AttitudeState;

void attitudeInit(AttitudeState *s);

void normalizeGravity(AttitudeState *s, float* acc);

void madgwickUpdate(AttitudeState *s, float* acc, float* gyro, uint8_t sz);

void computeAngles(AttitudeState *s);

float invSqrt(float x);

#endif /* INC_ATTITUDE_H_ */
