#ifndef _MPU6050_H
#define _MPU6050_H

#include <stdint.h>
#include <stdio.h>
#include "main.h"

#define MPU6050_ADDR (0x68 << 1)  // I2C address (shifted for HAL)
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define XA_OFFS_H 0x06
#define YA_OFFS_H 0x08
#define ZA_OFFS_H 0x0A
#define XG_OFFS_USRH 0x13
#define YG_OFFS_USRH 0x15
#define ZG_OFFS_USRH 0x17
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define SMPRT_DIV 0x19
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define FIFO_EN 0x23
#define USER_CTRL 0x6A

#define MPU6050_I2C &hi2c1

struct {
   float accelX;
   float accelY;
   float accelZ;
   float gyroX;
   float gyroY;
   float gyroZ;
   int16_t accelOffsets[3], gyroOffsets[3];
} typedef MPU6050_Data;

void MPU6050_Init(void);

void MPU6050_ReadAll(MPU6050_Data *dataToProcess);

void MPU6050_CalibrateInternal(MPU6050_Data *dataToProcess);

void MPU6050_CalibrateExternal(MPU6050_Data *dataToProcess);

void MPU6050_CalculateOffsets(int16_t *accelOffsets, int16_t *gyroOffsets);

void MPU6050_WriteOffsets(int16_t *accelOffsets, int16_t *gyroOffsets);

void MPU6050_SoftReset(void);

#endif
