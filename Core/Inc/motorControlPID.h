/*
 * motorControlPID.h
 *
 *  Created on: Jan 23, 2025
 *      Author: patryk
 */

#ifndef INC_MOTORCONTROLPID_H_
#define INC_MOTORCONTROLPID_H_

struct {
   float current_position;
   float target_position;
   float position_reached;
   float phase;
   float kp = 0.1f; // Proportional gain           //0.009 - 7.4V
   float ki = 0.0f; // Integral gain
   float kd = 0.0f; // Derivative gain
   float error = 0.0f, previous_error = 0.0f;        //0.4 0 0 - 5.5V
   float integral = 0.0f, derivative = 0.0f;
   float control_output = 0.0f;

   int16_t accelOffsets[3], gyroOffsets[3];
} typedef motorControlPID;

#endif /* INC_MOTORCONTROLPID_H_ */
