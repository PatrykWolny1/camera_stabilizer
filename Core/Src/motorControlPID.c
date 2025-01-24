/*
 * motorControlPID.c
 *
 *  Created on: Jan 23, 2025
 *      Author: patryk
 */

#include "motorControlPID.h"
#include "math.h"

void pwmInit(void) {
	HAL_TIM_PWM_Start(&htim2, MOTOR_PITCH_PWM_PIN_UL);
	HAL_TIM_PWM_Start(&htim2, MOTOR_PITCH_PWM_PIN_UH);
	HAL_TIM_PWM_Start(&htim1, MOTOR_PITCH_PWM_PIN_VL);
	HAL_TIM_PWM_Start(&htim1, MOTOR_PITCH_PWM_PIN_VH);
	HAL_TIM_PWM_Start(&htim1, MOTOR_PITCH_PWM_PIN_WL);
	HAL_TIM_PWM_Start(&htim1, MOTOR_PITCH_PWM_PIN_WH);
}

void pidInit(motorControlPID *motorControlPID, float kp, float kd, float ki) {
	motorControlPID->current_position = 0;
	motorControlPID->error = 0;
	motorControlPID->proportional = 0;
	motorControlPID->integral = 0;
	motorControlPID->integral_term = 0;
	motorControlPID->derivative = 0;
	motorControlPID->derivative_term = 0;
	motorControlPID->control_output = 0;
	motorControlPID->phase = 0;
	motorControlPID->previous_error = 0;

	motorControlPID->kp = kp;
	motorControlPID->kd = kd;
	motorControlPID->ki = ki;
}

void controlLoop(motorControlPID *motorControlPID, uint8_t motor_pitch_roll)
{
    // Calculate error
	motorControlPID->error = motorControlPID->target_position - motorControlPID->current_position;
//	printf("%.2f %.2f\r\n", motorControlPID->target_position, motorControlPID->current_position);
    // Check if the position is reached
    if (fabs(motorControlPID->error) < 0.01f) { // Error threshold
    	motorControlPID->position_reached = 1; // Mark position as stabilized
    } else {
    	motorControlPID->position_reached = 0; // Motor is still moving
    }

    // PID control
    motorControlPID->proportional = motorControlPID->kp * motorControlPID->error;

    motorControlPID->integral += motorControlPID->error;
    motorControlPID->integral_term = motorControlPID->ki * motorControlPID->integral;

    motorControlPID->derivative = motorControlPID->error - motorControlPID->previous_error;
    motorControlPID->derivative_term = motorControlPID->kd * motorControlPID->derivative;

    motorControlPID->control_output = motorControlPID->proportional + motorControlPID->integral_term;// + motorControlPID->derivative_term;

    // Update motor phase
    motorControlPID->phase += motorControlPID->control_output; // Adjust phase based on control output
    if (motorControlPID->phase >= TWO_PI) {
    	motorControlPID->phase -= TWO_PI;
    } else if (motorControlPID->phase < 0.0f) {
    	motorControlPID->phase += TWO_PI;
    }
    // Apply PWM signal
    setPWMDutyCyclePosition(motorControlPID->phase, motor_pitch_roll);

    // Update current position (simulated, replace with actual feedback if available)
    motorControlPID->current_position += motorControlPID->control_output / MOTOR_POLE_PAIRS;

    // Store previous error
    motorControlPID->previous_error = motorControlPID->error;
}

void setPWMDutyCyclePosition(float electrical_angle, uint8_t motor_pitch_roll) {
	   //Calculate sine values for each phase
//     float sin_u = 0.5f + 0.5f * sinf(electrical_angle);                // Phase U
//     float sin_v = 0.5f + 0.5f * sinf(electrical_angle - TWO_PI / 3.0); // Phase V
//     float sin_w = 0.5f + 0.5f * sinf(electrical_angle + TWO_PI / 3.0); // Phase W

    float amplitude = 1.0f; // Normalized amplitude (scale as needed)
    float sin3_alpha = sinf(3 * electrical_angle);

    // Calculate PWM values for each phase
    float sin_u = (1 / sqrtf(3)) * amplitude * (sinf(electrical_angle) + (1.0f / 6.0f) * sin3_alpha) + 0.5f;
    float sin_v = (1 / sqrtf(3)) * amplitude * (sinf(electrical_angle - TWO_PI / 3.0) + (1.0f / 6.0f) * sin3_alpha) + 0.5f;
    float sin_w = (1 / sqrtf(3)) * amplitude * (sinf(electrical_angle + TWO_PI / 3.0) + (1.0f / 6.0f) * sin3_alpha) + 0.5f;

//    printf("Duty U: %.2f, Duty V: %.2f, Duty W: %.2f\r\n", sin_u, sin_v, sin_w);

    // Convert sine values to PWM duty cycles
    uint16_t duty_u = (uint16_t)(sin_u * PWM_PERIOD);
    uint16_t duty_v = (uint16_t)(sin_v * PWM_PERIOD);
    uint16_t duty_w = (uint16_t)(sin_w * PWM_PERIOD);

    if (motor_pitch_roll == 0) {
		// Set high-side PWM channels
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_PITCH_PWM_PIN_UH, duty_u); // UH
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PITCH_PWM_PIN_VH, duty_v); // VH
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PITCH_PWM_PIN_WH, duty_w); // WH

		// Set low-side PWM channels
		__HAL_TIM_SET_COMPARE(&htim2, MOTOR_PITCH_PWM_PIN_UL, PWM_PERIOD - duty_u); // UL
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PITCH_PWM_PIN_VL, PWM_PERIOD - duty_v); // VL
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PITCH_PWM_PIN_WL, PWM_PERIOD - duty_w); // WL
	} else if (motor_pitch_roll == 1){
		printf("ROLL\r\n");
	}
}

