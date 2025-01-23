/*
 * motorRun.h
 *
 *  Created on: Jan 23, 2025
 *      Author: patryk
 */

#ifndef INC_MOTORRUN_H_
#define INC_MOTORRUN_H_


#define PWM_FREQUENCY 80000    // 20 kHz
#define ENABLE_PIN GPIO_PIN_0  // Motor driver enable pin
#define ENABLE_PORT GPIOC

#define MOTOR_POLE_PAIRS 7     // Number of pole pairs of the motor
#define PRESCALER_VALUE 1     // Prescaler value for 64 MHz clock
#define PWM_PERIOD 800        // ARR value for 20 kHz PWM with 64 MHz clock

#define TWO_PI 6.28318530718f  // 2 * pi for sine calculation
#define M_PI 3.14159265359f    // pi constant

#define PWM_PIN_UL GPIO_PIN_15
#define PWM_PIN_UH GPIO_PIN_3
#define PWM_PIN_VL GPIO_PIN_10
#define PWM_PIN_VH GPIO_PIN_11
#define PWM_PIN_WL GPIO_PIN_6
#define PWM_PIN_WH GPIO_PIN_7



#endif /* INC_MOTORRUN_H_ */
