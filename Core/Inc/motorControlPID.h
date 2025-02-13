/*
 * motorControlPID.h
 *
 *  Created on: Jan 23, 2025
 *      Author: patryk
 */

#ifndef INC_MOTORCONTROLPID_H_
#define INC_MOTORCONTROLPID_H_

#include "main.h"
#define PWM_FREQUENCY 80000    // 80 kHz
#define ENABLE_PIN GPIO_PIN_0  // Motor driver enable pin
#define ENABLE_PORT GPIOC

#define MOTOR_POLE_PAIRS 7     // Number of pole pairs of the motor
#define PRESCALER_VALUE 1     // Prescaler value for 64 MHz clock
#define PWM_PERIOD 800        // ARR value for 20 kHz PWM with 64 MHz clock

#define TWO_PI 6.28318530718f  // 2 * pi for sine calculation

#define MOTOR_PITCH_PWM_PIN_UL TIM_CHANNEL_1	//PA0 TIM2
#define MOTOR_PITCH_PWM_PIN_UH TIM_CHANNEL_2	//PA1 TIM2
#define MOTOR_PITCH_PWM_PIN_VL TIM_CHANNEL_1 	//PA8 TIM1
#define MOTOR_PITCH_PWM_PIN_VH TIM_CHANNEL_2 	//PA9 TIM1
#define MOTOR_PITCH_PWM_PIN_WL TIM_CHANNEL_3	//PA10 TIM1
#define MOTOR_PITCH_PWM_PIN_WH TIM_CHANNEL_4 	//PA11 TIM1

#define MOTOR_ROLL_PWM_PIN_UL TIM_CHANNEL_1		//PB4 TIM3
#define MOTOR_ROLL_PWM_PIN_UH TIM_CHANNEL_2		//PB5 TIM3
#define MOTOR_ROLL_PWM_PIN_VL TIM_CHANNEL_1 	//PC6 TIM8
#define MOTOR_ROLL_PWM_PIN_VH TIM_CHANNEL_2 	//PC7 TIM8
#define MOTOR_ROLL_PWM_PIN_WL TIM_CHANNEL_3		//PC8 TIM8
#define MOTOR_ROLL_PWM_PIN_WH TIM_CHANNEL_4 	//PC9 TIM8

struct {
   float current_position;
   float target_position;
   float position_reached;
   float phase;
   float kp; 			// Proportional gain 0.009 - 7.4V
   float ki; 			// Integral gain
   float kd; 			// Derivative gain
   float error;
   float previous_error;
   float proportional;
   float integral;
   float derivative;
   float integral_term;
   float derivative_term;
   float control_output;
} typedef motorControlPID;

void pwmInit(void);
void controlLoop(motorControlPID *motorControlPID, uint8_t motor_pitch_roll);
void setPWMDutyCyclePosition(float electrical_angle, uint8_t motor_pitch_roll);

#endif /* INC_MOTORCONTROLPID_H_ */
