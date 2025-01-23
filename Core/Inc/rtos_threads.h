/*
 * threads.h
 *
 *  Created on: Dec 19, 2024
 *      Author: patryk
 */

#ifndef INC_RTOS_THREADS_H_
#define INC_RTOS_THREADS_H_

#include "oled.h"
#include "write_printf.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <unistd.h> // For getcwd
#include <stdlib.h>  // For malloc/free

void mpu6050_ReadData(void);
void DataProcessing(void);

#endif /* INC_RTOS_THREADS_H_ */
