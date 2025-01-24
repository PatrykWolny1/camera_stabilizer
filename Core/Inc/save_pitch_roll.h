/*
 * save_pitch_roll.h
 *
 *  Created on: Jan 24, 2025
 *      Author: patryk
 */

#include "main.h"
#include "stm32f4xx_hal.h"

#define FLASH_SECTOR FLASH_SECTOR_7
#define FLASH_SECTOR FLASH_SECTOR_7
#define FLASH_SECTOR_BASE 0x08060000   // Base address for Sector 7

#define PITCH_ADDRESS (FLASH_SECTOR_BASE + 0x0000) // First address in Sector 7
#define ROLL_ADDRESS  (FLASH_SECTOR_BASE + 0x0004)  // Next address, 4 bytes apart

void EraseSector(void);
void WriteFloatToFlash(float data, uint32_t address);
float ReadFloatFromFlash(uint32_t address);
