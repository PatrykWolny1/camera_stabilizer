	/*
 * threads.cpp
 *
 *  Created on: Dec 19, 2024
 *      Author: patryk
 */


#include "rtos_threads.h"
#include "mpu6050.h"
#include "compute_angles.h"
#include "motorControlPID.h"
#include "ssd1306.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <unistd.h> // For getcwd
#include <stdlib.h>  // For malloc/free
#include <string.h>

extern osMessageQueueId_t MPU6050DataHandle;
extern osMessageQueueId_t KalmanAngleHandle;
extern osMutexId_t uartMutexHandle;
extern osSemaphoreId_t dmaTxCompleteSemaphoreHandle;

void mpu6050_ReadData(void *argument) {
	uint8_t transmitViaUART = 0;
    MPU6050_Data dataToProcess;
    char buffer[200];

	printf("OLED initialization...\r\n");
    SSD1306_Init();
	printf("OLED initialized\r\n");

	SSD1306_DrawString(14, 28, "MPU6050 loading...", 1);
	SSD1306_UpdateScreen();

    MPU6050_Init();
    printf("MPU6050 initialized\r\n");

    printf("MPU6050 calibrating...\r\n");
    //MPU6050_CalibrateInternal(&dataToProcess);
    MPU6050_CalibrateExternal(&dataToProcess);
    printf("MPU6050 calibrated\r\n");

    SSD1306_Clear();

    while (1) {
        // Read sensor data
        MPU6050_ReadAll(&dataToProcess);

        if (transmitViaUART) {
			//Format sensor data
			snprintf(buffer, sizeof(buffer),
					 "Xa=%6.2f Xg=%6.2f Ya=%6.2f Yg=%6.2f Za=%6.2f Zg=%6.2f\r\n",
					 dataToProcess.accelX, dataToProcess.gyroX, dataToProcess.accelY,
					 dataToProcess.gyroY, dataToProcess.accelZ, dataToProcess.gyroZ);

			//Transmit data over UART
			if (osMutexAcquire(uartMutexHandle, 100) == osOK) {  // Use timeout to avoid deadlocks
				if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buffer, strlen(buffer)) == HAL_OK) {
					if (osSemaphoreAcquire(dmaTxCompleteSemaphoreHandle, 100) != osOK) {
						printf("UART DMA timeout\r\n");
					}
				} else {
					printf("UART DMA transmission failed\r\n");
				}
				osMutexRelease(uartMutexHandle);  // Release mutex
			} else {
				printf("UART mutex acquire failed\r\n");
			}
        }

        // Send data to the queue
        if (osMessageQueuePut(MPU6050DataHandle, &dataToProcess, 0, 200) != osOK) {
            printf("Queue is full\r\n");
        }

        osDelay(10);  // Prevent rapid polling
    }
}


void DataProcessing(void *argument) {
    MPU6050_Data receivedData;
    PitchRollYaw *resultsPRY;

    CompFilter *resultsCompFilter;

    KalmanFilter resultKalmanFilterPitch;
    KalmanFilter resultKalmanFilterRoll;
    float kalmanRoll;
    float kalmanPitch;
    double dt;

    uint8_t transmitViaUART = 0;
    uint8_t chooseFilter = 1;

    char buffer[200];

    printf("DataProcessing task started\r\n");

    Kalman_Init(&resultKalmanFilterPitch, 0.006f, 0.003f, 0.000640f); // Initialize the Kalman filter with noise parameters
    Kalman_Init(&resultKalmanFilterRoll, 0.006f, 0.003f, 0.000640f); // Initialize the Kalman filter with noise parameters
    uint32_t prevTick = osKernelGetTickCount();  // Initialize previous tick

    while (1) {

        // Get data from queue
        if (osMessageQueueGet(MPU6050DataHandle, &receivedData, NULL, 200) == osOK) {
        	// Record the start time
            SSD1306_DrawString(34, 16, "Pitch", 1); // Label for pitch
            SSD1306_DrawString(74, 16, "Roll", 1); // Label for pitch

            //resultsCompFilter = complementary_filter(&resultsPRY, &receivedData, &prevTick);
            //resultsPRY = computeAnglesAcc(&receivedData);
            resultsPRY = computeAngles(&receivedData, &prevTick);
            // Clear regions where pitch and roll values are displayed
            SSD1306_ClearRegion(34, 28, 40, 8); // Clear the region for pitch value
            SSD1306_ClearRegion(74, 28, 40, 8); // Clear the region for roll value

            dt = computeDT(&prevTick);
            kalmanPitch = Kalman_Update(&resultKalmanFilterPitch, resultsPRY->pitchAcc, receivedData.gyroX, (float)dt);
            kalmanRoll = Kalman_Update(&resultKalmanFilterRoll, resultsPRY->rollAcc, receivedData.gyroY, (float)dt);
             //SSD1306_DrawFloat(29, 28, resultsCompFilter->pitch, 1, 1);
            SSD1306_DrawFloat(34, 28, kalmanPitch, 1, 1);

            //SSD1306_DrawFloat(69, 28, resultsCompFilter->roll, 1, 1);
            SSD1306_DrawFloat(74, 28, kalmanRoll, 1, 1);

            SSD1306_UpdateScreen();

            //Format processed data for computeAngles()


            if (transmitViaUART) {
            	if (chooseFilter == 0) {
					snprintf(buffer, sizeof(buffer), "Acc | Pitch=%6.2f Roll=%6.2f | Gyro | Pitch=%6.2f Roll=%6.2f"
							"Yaw=%6.2f\r\n", resultsPRY->pitchAcc, resultsPRY->rollAcc, resultsPRY->pitchGyro,
							resultsPRY->rollGyro, resultsPRY->yawGyro);
            	} else if (chooseFilter == 1) {
					//Format processed data for filters
					snprintf(buffer, sizeof(buffer), "CompFilter | Pitch=%6.2f Roll=%6.2f\r\n",
							resultsCompFilter->pitch, resultsCompFilter->roll);
            	} else if (chooseFilter == 2) {
					//Format processed data for filters
					snprintf(buffer, sizeof(buffer), "%6.2f|%6.2f|%6.2f|%6.2f|%6.2f|%6.2f\r\n",
							kalmanPitch, kalmanRoll, resultsPRY->pitchAcc, resultsPRY->rollAcc,
							resultsPRY->pitchGyro, resultsPRY->rollGyro);
            	} else {
            		printf("No filter chosen!");
            	}
				//Transmit processed data over UART
				if (osMutexAcquire(uartMutexHandle, 100) == osOK) {  // Use timeout to avoid deadlocks
					if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buffer, strlen(buffer)) == HAL_OK) {
						if (osSemaphoreAcquire(dmaTxCompleteSemaphoreHandle, 100) != osOK) {
							printf("UART DMA timeout\r\n");
						}
					} else {
						printf("UART DMA transmission failed\r\n");
					}
					osMutexRelease(uartMutexHandle);  // Release mutex
				} else {
					printf("UART mutex acquire failed\r\n");
				}
            }
    		osDelay(5);  // Prevent rapid polling
        } else {
        	//printf("Queue is empty\r\n");
        }
    }
}

void motorRun(void *argument) {

	motorControlPID *motorControlPID;

	// Start PWM on all channels

//	pwmInit();

	while (1) {
		/* USER CODE END WHILE */
		// Update PWM duty cycle for open-loop control

		// Increment phase
		// phase += motor_speed * TWO_PI / 1000.0f; // Increment based on speed
		// // printf("Phase: %.2f\r\n", phase);
		// if (phase >= TWO_PI) {
		//     phase -= TWO_PI; // Wrap phase
		// }
		// // Calculate the electrical angle for the target position
		// set_pwm_duty_cycle(phase, motor_speed);
		// delay_us(&htim4, 170); // 1 ms delay for ~1 kHz update rate
		// Calculate error

		// Check user button
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
			// Increment target position by 1.57 rad (90 degrees)


			// Wait until button is released (debounce)
			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);

			// Wait for the motor to stabilize
			// wait_for_stabilization();
//			motorControlPID->current_position = 0;
//			motorControlPID->target_position = 1.57f;
		}
		// Continue control loop
//		controlLoop(motorControlPID, 0);

		// Delay for loop timing
		osDelay(5);
	}
}


