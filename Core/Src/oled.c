/*
 * oled.c
 *
 *  Created on: Dec 23, 2024
 *      Author: patryk
 */

#include "main.h"
#include "oled.h"

// I2C handle

void OLED_Init(void) {
	if (HAL_SPI_Transmit(&hspi1, 0xAA, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("SPI transmission error\r\n");
	}

	SSD1306_Init();
	HAL_Delay(10);
}
