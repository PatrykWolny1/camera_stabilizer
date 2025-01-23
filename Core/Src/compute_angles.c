/*
 * compute_angles.c
 *
 *  Created on: Dec 22, 2024
 *      Author: patryk
 */

#include "compute_angles.h"

PitchRollYaw resultsPRY;
CompFilter resultsCompFilter;

double computeDT(uint32_t *prevTick) {
    uint32_t currTick = osKernelGetTickCount();  // Get current tick
    uint32_t tickFreq = osKernelGetTickFreq();  // Get tick frequency (e.g., 1000 Hz)

    // Ensure tick frequency is valid
    if (tickFreq == 0) {
        printf("Error: Tick frequency is zero!\n");
        return 0.0;  // Avoid division by zero
    }

    uint32_t deltaTicks;
    if (currTick >= *prevTick) {
        deltaTicks = currTick - *prevTick;  // Normal case
    } else {
        deltaTicks = (0xFFFFFFFF - *prevTick) + currTick + 1;  // Handle overflow
    }

    *prevTick = currTick;  // Update previous tick for the next call

    double dt = (double)deltaTicks / tickFreq;  // Convert ticks to seconds

    // Debugging output
//    printf("computeDT -> currTick: %lu, prevTick: %lu, deltaTicks: %lu, tickFreq: %lu, dt: %.6f\n",
//           currTick, *prevTick, deltaTicks, tickFreq, dt);

    return dt;  // Return time difference in seconds
}

float radiansToDegrees(float radians) {
    return radians * (180.0f / M_PI);
}

CompFilter* complementary_filter(PitchRollYaw *resultsPRY, MPU6050_Data *dataToProcess, uint32_t *prevTick) {
	float alpha = 0.982;  // Weight factor (typically close to 1)
	resultsPRY->pitchAcc = atan2(dataToProcess->accelY, sqrt(pow(dataToProcess->accelX,2)+pow(dataToProcess->accelZ,2)));
	resultsPRY->rollAcc = atan2(dataToProcess->accelX, sqrt(pow(dataToProcess->accelY,2)+pow(dataToProcess->accelZ,2)));

    resultsPRY->pitchAcc = radiansToDegrees(resultsPRY->pitchAcc);
    resultsPRY->rollAcc = radiansToDegrees(resultsPRY->rollAcc);

    double dt = computeDT(prevTick);

	// Complementary filter equation
	resultsCompFilter.pitch = alpha * (resultsCompFilter.pitch + dataToProcess->gyroX * dt) + (1 - alpha) * resultsPRY->pitchAcc;
	resultsCompFilter.roll = alpha * (resultsCompFilter.roll + dataToProcess->gyroY * dt) + (1 - alpha) * resultsPRY->rollAcc;

	return &resultsCompFilter;
}

// Initialize the Kalman filter

//The Measurement Noise (R) for the Kalman filter, based on the accelerometer noise density of 400 ug/sqrt(Hz) at 8kHz sample rate is R=0.000640
void Kalman_Init(KalmanFilter *kf, float Q_angle, float Q_bias, float R_measure) {
    // Initialization of states
    kf->angle = 0.0f; // θ₀|₀ = 0
    kf->bias = 0.0f;  // b₀|₀ = 0
    kf->rate = 0.0f;  // Unbiased angular rate is initialized to 0

    // Initialize the error covariance matrix P₀|₀
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;

    // Set process and measurement noise covariances
    kf->Q_angle = Q_angle;    // Q_θ: Variance in angle
    kf->Q_bias = Q_bias;      // Q_b: Variance in gyroscope bias
    kf->R_measure = R_measure; // R: Variance in accelerometer measurement
}

// Update the Kalman filter with new sensor data
float Kalman_Update(KalmanFilter *kf, float new_angle, float new_rate, float dt) {
    // Step 1: Prediction

    // Predict the unbiased rate: ωₖ - bₖ
    kf->rate = new_rate - kf->bias; // Equation: ωₖ - bₖ

    // Predict the angle: θₖ|ₖ₋₁ = θₖ₋₁|ₖ₋₁ + Δt * (ωₖ - bₖ)
    kf->angle += dt * kf->rate; // θₖ|ₖ₋₁ = θₖ₋₁|ₖ₋₁ + Δt * rate

    // Update the error covariance matrix:
    // Pₖ|ₖ₋₁ = F * Pₖ₋₁|ₖ₋₁ * Fᵀ + Q
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle); // Update Pₖ|ₖ₋₁[0][0]
    kf->P[0][1] -= dt * kf->P[1][1]; // Update Pₖ|ₖ₋₁[0][1]
    kf->P[1][0] -= dt * kf->P[1][1]; // Update Pₖ|ₖ₋₁[1][0]
    kf->P[1][1] += kf->Q_bias * dt;  // Update Pₖ|ₖ₋₁[1][1]

    // Step 2: Update

    // Compute the innovation: yₖ = zₖ - H * θₖ|ₖ₋₁
    float y = new_angle - kf->angle; // Innovation: yₖ = zₖ - θₖ|ₖ₋₁

    // Compute the innovation covariance: S = H * Pₖ|ₖ₋₁ * Hᵀ + R
    float S = kf->P[0][0] + kf->R_measure; // S = Pₖ|ₖ₋₁[0][0] + R

    // Compute the Kalman gain: K = Pₖ|ₖ₋₁ * Hᵀ / S
    float K[2];
    K[0] = kf->P[0][0] / S; // K₁ = Pₖ|ₖ₋₁[0][0] / S
    K[1] = kf->P[1][0] / S; // K₂ = Pₖ|ₖ₋₁[1][0] / S

    // Update the state estimates: θₖ|ₖ = θₖ|ₖ₋₁ + K₁ * yₖ
    kf->angle += K[0] * y; // θₖ|ₖ = θₖ|ₖ₋₁ + K₁ * yₖ

    // Update the bias: bₖ|ₖ = bₖ|ₖ₋₁ + K₂ * yₖ
    kf->bias += K[1] * y; // bₖ|ₖ = bₖ|ₖ₋₁ + K₂ * yₖ

    // Update the error covariance matrix:
    // Pₖ|ₖ = (I - K * H) * Pₖ|ₖ₋₁
    float P00_temp = kf->P[0][0]; // Temporary value to hold Pₖ|ₖ₋₁[0][0]
    float P01_temp = kf->P[0][1]; // Temporary value to hold Pₖ|ₖ₋₁[0][1]

    kf->P[0][0] -= K[0] * P00_temp; // Update Pₖ|ₖ[0][0]
    kf->P[0][1] -= K[0] * P01_temp; // Update Pₖ|ₖ[0][1]
    kf->P[1][0] -= K[1] * P00_temp; // Update Pₖ|ₖ[1][0]
    kf->P[1][1] -= K[1] * P01_temp; // Update Pₖ|ₖ[1][1]

    // Return the updated angle estimate
    return kf->angle; // Return θₖ|ₖ
}


PitchRollYaw* computeAngles(MPU6050_Data *dataToProcess, uint32_t *prevTick) {
	resultsPRY.pitchAcc = atan2(dataToProcess->accelY, sqrt(pow(dataToProcess->accelX,2)+pow(dataToProcess->accelZ,2)));
	resultsPRY.rollAcc = atan2(dataToProcess->accelX, sqrt(pow(dataToProcess->accelY,2)+pow(dataToProcess->accelZ,2)));

    resultsPRY.pitchAcc = radiansToDegrees(resultsPRY.pitchAcc);
    resultsPRY.rollAcc = radiansToDegrees(resultsPRY.rollAcc);

    double dt = computeDT(prevTick);

    resultsPRY.pitchGyroDelta = dataToProcess->gyroX * dt;
    resultsPRY.rollGyroDelta = dataToProcess->gyroY * dt;
    resultsPRY.yawGyroDelta = dataToProcess->gyroZ * dt;

    resultsPRY.pitchGyro += resultsPRY.pitchGyroDelta;
    resultsPRY.rollGyro += resultsPRY.rollGyroDelta;
    resultsPRY.yawGyro += resultsPRY.yawGyroDelta;

    return &resultsPRY;
}


PitchRollYaw* computeAnglesAcc(MPU6050_Data *dataToProcess) {
	resultsPRY.pitchAcc = atan2(dataToProcess->accelY, sqrt(pow(dataToProcess->accelX,2)+pow(dataToProcess->accelZ,2)));
	resultsPRY.rollAcc = atan2(dataToProcess->accelX, sqrt(pow(dataToProcess->accelY,2)+pow(dataToProcess->accelZ,2)));

    resultsPRY.pitchAcc = radiansToDegrees(resultsPRY.pitchAcc);
    resultsPRY.rollAcc = radiansToDegrees(resultsPRY.rollAcc);

    return &resultsPRY;
}

