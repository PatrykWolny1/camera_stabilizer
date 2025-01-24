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
#include "save_pitch_roll.h"
#include "ssd1306.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <unistd.h> // For getcwd
#include <stdlib.h>  // For malloc/free
#include <string.h>
#include "core_cm4.h" // Include appropriate core header file, might differ based on your Cortex version

void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) { // Check if cycle counter is enabled
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable cycle counter
        DWT->CYCCNT = 0; // Reset the cycle counter
    }
}

void Delay_us(uint32_t microseconds) {
    uint32_t startTick = DWT->CYCCNT,
             delayTicks = microseconds * (SystemCoreClock/1000000);
    while (DWT->CYCCNT - startTick < delayTicks);
}

extern osMessageQueueId_t MPU6050DataHandle;
extern osMessageQueueId_t KalmanAngleHandle;
extern osMutexId_t uartMutexHandle;

extern osSemaphoreId_t dmaTxCompleteSemaphoreHandle;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

motorControlPID motorControlPitch;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint32_t last_interrupt_time = 0;
	uint32_t current_time = HAL_GetTick(); // Get current time in ms

	if (GPIO_Pin == GPIO_PIN_13) {
	    // Write pitch and roll to flash
//		WriteFloatToFlash(motorControlPitch.current_position, PITCH_ADDRESS);
//	    WriteFloatToFlash(kalmanRoll, ROLL_ADDRESS);
		if ((current_time - last_interrupt_time) > 40) { // Debounce time: 50 ms
			motorControlPitch.target_position += 1.57f;
			if (motorControlPitch.target_position > 4 * 1.57) {
				motorControlPitch.current_position = 0;
				motorControlPitch.target_position = 1.57f;
			}
		}
		last_interrupt_time = current_time;
	}
}

void mpu6050_ReadData(void *argument) {

	uint8_t rawTransmitViaUART = 0;
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

    uint32_t tick = osKernelGetTickCount();  // Get the current tick count

    while (1) {
        tick += osKernelGetTickFreq() / 100;  // Calculate next wake time (10ms)

        // Read sensor data
        MPU6050_ReadAll(&dataToProcess);


        if (rawTransmitViaUART) {
            // Format sensor data
            snprintf(buffer, sizeof(buffer),
                     "Xa=%6.2f Xg=%6.2f Ya=%6.2f Yg=%6.2f Za=%6.2f Zg=%6.2f\r\n",
                     dataToProcess.accelX, dataToProcess.gyroX, dataToProcess.accelY,
                     dataToProcess.gyroY, dataToProcess.accelZ, dataToProcess.gyroZ);

            // Transmit data over UART
            if (osMutexAcquire(uartMutexHandle, 100) == osOK) {
                if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buffer, strlen(buffer)) == HAL_OK) {
                    if (osSemaphoreAcquire(dmaTxCompleteSemaphoreHandle, 100) != osOK) {
                        printf("UART DMA timeout\r\n");
                    }
                } else {
                    printf("UART DMA transmission failed\r\n");
                }
                osMutexRelease(uartMutexHandle);
            } else {
                printf("UART mutex acquire failed\r\n");
            }
        }
        // Send data to the queue
        if (osMessageQueuePut(MPU6050DataHandle, &dataToProcess, 0, 200) != osOK) {
            printf("Queue is full\r\n");
        }
        osDelayUntil(tick);  // Delay until the next calculated tick

//        osDelay(10);  // Prevent rapid polling
    }
}


void DataProcessing(void *argument) {
    MPU6050_Data receivedData;
    PitchRollYaw *resultsPRY;

    CompFilter *resultsCompFilter;

    KalmanFilter resultKalmanFilterPitch;
    KalmanFilter resultKalmanFilterRoll;

    float kalmanPitch;
    float kalmanRoll;

    double dt;

    uint8_t transmitViaUART = 0;
    uint8_t chooseFilter = 2;		//RawAngle - 0 | Complementary filter - 1 | Kalman filter - 2

    char buffer[200];

    printf("Data processing task started\r\n");

    Kalman_Init(&resultKalmanFilterPitch, 0.006f, 0.003f, 0.000640f); // Initialize the Kalman filter with noise parameters
    Kalman_Init(&resultKalmanFilterRoll, 0.006f, 0.003f, 0.000640f); // Initialize the Kalman filter with noise parameters
    uint32_t prevTick = osKernelGetTickCount();  // Initialize previous tick
    uint32_t tick = osKernelGetTickCount();  // Get the current tick count

    while (1) {
        tick += osKernelGetTickFreq() / 50;   // Calculate next wake time (20ms)

        // Get data from queue
        if (osMessageQueueGet(MPU6050DataHandle, &receivedData, NULL, 200) == osOK) {
        	// Record the start time
            SSD1306_DrawString(34, 16, "Pitch", 1); // Label for pitch
            SSD1306_DrawString(74, 16, "Roll", 1); // Label for pitch

            // Clear regions where pitch and roll values are displayed
            SSD1306_ClearRegion(34, 28, 40, 8); // Clear the region for pitch value
            SSD1306_ClearRegion(74, 28, 40, 8); // Clear the region for roll value

            if (chooseFilter == 0) {
                resultsPRY = computeAngles(&receivedData, &prevTick);
            } else if (chooseFilter == 1) {
                resultsPRY = computeAngles(&receivedData, &prevTick);
                resultsCompFilter = complementary_filter(resultsPRY, &receivedData, &prevTick);
                SSD1306_DrawFloat(34, 28, resultsCompFilter->pitch, 1, 1);
                SSD1306_DrawFloat(74, 28, resultsCompFilter->roll, 1, 1);
            } else if (chooseFilter == 2) {
				resultsPRY = computeAnglesAcc(&receivedData);
				dt = computeDT(&prevTick);
				kalmanPitch = Kalman_Update(&resultKalmanFilterPitch, resultsPRY->pitchAcc, receivedData.gyroX, (float)dt);
				kalmanRoll = Kalman_Update(&resultKalmanFilterRoll, resultsPRY->rollAcc, receivedData.gyroY, (float)dt);
	            SSD1306_DrawFloat(34, 28, kalmanPitch, 1, 1);
				SSD1306_DrawFloat(74, 28, kalmanRoll, 1, 1);
            }
            SSD1306_UpdateScreen();

            if (transmitViaUART) {
            	if (chooseFilter == 0) {
					snprintf(buffer, sizeof(buffer), "Acc | Pitch=%6.2f Roll=%6.2f | Gyro | Pitch=%6.2f Roll=%6.2f"
							"Yaw=%6.2f\r\n", resultsPRY->pitchAcc, resultsPRY->rollAcc, resultsPRY->pitchGyro,
							resultsPRY->rollGyro, resultsPRY->yawGyro);
            	} else if (chooseFilter == 1) {
					//Complementary filter
					snprintf(buffer, sizeof(buffer), "CompFilter | Pitch=%6.2f Roll=%6.2f\r\n",
							resultsCompFilter->pitch, resultsCompFilter->roll);
            	} else if (chooseFilter == 2) {
            		//Kalman filter
					snprintf(buffer, sizeof(buffer), "%6.2f|%6.2f|%6.2f|%6.2f|%6.2f|%6.2f\r\n",
							kalmanPitch, kalmanRoll, resultsPRY->pitchAcc, resultsPRY->rollAcc,
							resultsPRY->pitchGyro, resultsPRY->rollGyro);
            	} else {
            		printf("No filter chosen!");
            	}
				//Transmit processed data over UART
				if (osMutexAcquire(uartMutexHandle, 200) == osOK) {  // Use timeout to avoid deadlocks
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
            if (osMessageQueuePut(KalmanAngleHandle, &kalmanPitch, 0, 200) != osOK) {
                printf("Queue is full\r\n");
            }
            osDelayUntil(tick);

//    		osDelay(20);  // Prevent rapid polling
        } else {
        	//printf("Queue is empty\r\n");
        }
    }
}

void motorRun(void *argument) {
	// Initialization of DWT
	DWT_Init();
	printf("Motor control started\r\n");
	// Start PWM on all channels
//	motorControlPID motorControlPitch;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Set PB1 high

	pwmInit();
	pidInit(&motorControlPitch, 1.1, 0.01, 0.13);	//1.1, 0.01, 0.13 - 0.1s | 0.8, 2.88, 0.3 - 0.1s

	// Read back to verify
//	float savedPitch = ReadFloatFromFlash(PITCH_ADDRESS);
//	float savedRoll = ReadFloatFromFlash(ROLL_ADDRESS);

	float kalmanPitch = 0;
	float kalmanRoll = 0;
	motorControlPitch.target_position = 1.57f;

	float phase = 0;
	float motor_speed = 25;
    uint32_t tick = osKernelGetTickCount();

	while (1) {
        tick += osKernelGetTickFreq() / 125;  // Calculate next wake time (8ms)

		/* USER CODE END WHILE */
		// Update PWM duty cycle for open-loop control

		//Increment phase
//		 phase += motor_speed * TWO_PI / 100.0f; // Increment based on speed
//		 printf("Phase: %.2f\r\n", phase);
//		 if (phase >= TWO_PI) {
//		     phase -= TWO_PI; // Wrap phase
//		 }
//		setPWMDutyCyclePosition(phase, 1);
        if (osMessageQueueGet(KalmanAngleHandle, &kalmanPitch, NULL, 200) == osOK) {
        	motorControlPitch.target_position = -kalmanPitch * (M_PI / 180.0f); ;

			controlLoop(&motorControlPitch, 0);
			osDelayUntil(tick);

			// Delay for loop timing
			//osDelay(8);
        } else {
        	//printf("Queue is empty\r\n");
        }
	}
}


