/*
 * compute_angles.h
 *
 *  Created on: Dec 22, 2024
 *      Author: patryk
 */
#ifndef _COMPUTE_ANG_H
#define _COMPUTE_ANG_H

#include "math.h"
#include <stdint.h>
#include "mpu6050.h"

// Kalman filter state variables
typedef struct {
    float angle;    // Estimated angle (θₖ|ₖ)
    float bias;     // Estimated gyroscope bias (bₖ|ₖ)
    float rate;     // Unbiased gyroscope rate (ωₖ - bₖ)

    float P[2][2];  // Error covariance matrix (Pₖ|ₖ)
    float Q_angle;  // Process noise variance for the angle (Q_θ)
    float Q_bias;   // Process noise variance for the bias (Q_b)
    float R_measure; // Measurement noise variance (R)
} KalmanFilter;

struct {
    float pitchAcc;
	float rollAcc;
    float pitchGyro;
    float rollGyro;
    float yawGyro;
    float pitchGyroDelta;
    float rollGyroDelta;
    float yawGyroDelta;
} typedef PitchRollYaw;

struct {
	float alpha;
	float pitch;
	float roll;
} typedef CompFilter;

double computeDT(uint32_t *prevTick);
void Kalman_Init(KalmanFilter *kf, float Q_angle, float Q_bias, float R_measure);
float Kalman_Update(KalmanFilter *kf, float new_angle, float new_rate, float dt);
PitchRollYaw* computeAngles(MPU6050_Data *dataToProcess, uint32_t *prevTick);
CompFilter* complementary_filter(PitchRollYaw *resultsPRY, MPU6050_Data *dataToProcess, uint32_t *prevTick);

#endif
