/*
 * motorControlPID.c
 *
 *  Created on: Jan 23, 2025
 *      Author: patryk
 */

#include "motorControlPID.h"

void control_loop(void)
{
    // Calculate error
    error = target_position - current_position;

    // Check if the position is reached
    if (fabs(error) < 0.01f) { // Error threshold
        position_reached = 1; // Mark position as stabilized
    } else {
        position_reached = 0; // Motor is still moving
    }

    // PID control
    float proportional = kp * error;
    integral += error; // Accumulate integral
    float integral_term = ki * integral;
    derivative = error - previous_error;
    float derivative_term = kd * derivative;
    control_output = proportional + integral_term + derivative_term;

    // Update motor phase
    phase += control_output; // Adjust phase based on control output
    if (phase >= TWO_PI) {
        phase -= TWO_PI;
    } else if (phase < 0.0f) {
        phase += TWO_PI;
    }

    // Apply PWM signal
    set_pwm_duty_cycle_position(phase);

    // Update current position (simulated, replace with actual feedback if available)
    current_position += control_output / MOTOR_POLE_PAIRS;

    // Store previous error
    previous_error = error;
}

void set_pwm_duty_cycle_position(float electrical_angle) {
    // // Calculate sine values for each phase
    // float sin_u = 0.5f + 0.5f * sinf(electrical_angle);                // Phase U
    // float sin_v = 0.5f + 0.5f * sinf(electrical_angle - TWO_PI / 3.0); // Phase V
    // float sin_w = 0.5f + 0.5f * sinf(electrical_angle + TWO_PI / 3.0); // Phase W

    float amplitude = 1.0f; // Normalized amplitude (scale as needed)
    float sin3_alpha = sinf(3 * electrical_angle);

    // Calculate PWM values for each phase
    float sin_u = (1 / sqrtf(3)) * amplitude * (sinf(electrical_angle) + (1.0f / 6.0f) * sin3_alpha) + 0.5f;
    float sin_v = (1 / sqrtf(3)) * amplitude * (sinf(electrical_angle - TWO_PI / 3.0) + (1.0f / 6.0f) * sin3_alpha) + 0.5f;
    float sin_w = (1 / sqrtf(3)) * amplitude * (sinf(electrical_angle + TWO_PI / 3.0) + (1.0f / 6.0f) * sin3_alpha) + 0.5f;

    // printf("Duty U: %.2f, Duty V: %.2f, Duty W: %.2f\r\n", sin_u, sin_v, sin_w);

    // Convert sine values to PWM duty cycles
    uint16_t duty_u = (uint16_t)(sin_u * PWM_PERIOD);
    uint16_t duty_v = (uint16_t)(sin_v * PWM_PERIOD);
    uint16_t duty_w = (uint16_t)(sin_w * PWM_PERIOD);

    // Set high-side PWM channels
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_u); // UH
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_v); // VH
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_w); // WH

    // Set low-side PWM channels
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_PERIOD - duty_u); // UL
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_PERIOD - duty_v); // VL
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_PERIOD - duty_w); // WL
}
