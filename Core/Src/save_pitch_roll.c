/*
 * save_pitch_roll.c
 *
 *  Created on: Jan 24, 2025
 *      Author: patryk
 */

#include "save_pitch_roll.h"

void EraseSector(void) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR;
    EraseInitStruct.NbSectors = 1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        // Error handling
    }

    HAL_FLASH_Lock();
}

void WriteFloatToFlash(float data, uint32_t address) {
    HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *(uint32_t*)&data) != HAL_OK) {
        // Error handling
    }

    HAL_FLASH_Lock();
}

float ReadFloatFromFlash(uint32_t address) {
    return *(float*)address;
}
