/*
 * mpu6050.c
 *
 *  Created on: Dec 20, 2024
 *      Author: patryk
 */

#include "mpu6050.h"
#include "write_printf.h"
#include "main.h"

extern MPU6050_Data dataToProcess;

void MPU6050_Init(void) {
    uint8_t data[2];

//    MPU6050_SoftReset();

    HAL_Delay(2000);

    // Wake up the MPU6050 (write 0 to PWR_MGMT_1 register)
    data[0] = 0x6B;  // Register address
    data[1] = 0x00;  // Data to write
    HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		uint32_t err = HAL_I2C_GetError(&hi2c1);
		printf("I2C transmit failed, status: %d, error: 0x%08lx\r\n", status, err);
	} else {
		printf("I2C transmit succeeded.\r\n");
	}

    uint8_t data_config;

    // Set accelerometer range to ±2g
    data_config = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &data_config, 1, HAL_MAX_DELAY);

    // Set gyroscope range to ±250°/s
    data_config = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &data_config, 1, HAL_MAX_DELAY);

    uint8_t config;
    // Set DLPF_CFG to 0x02 (21 Hz accelerometer, 20 Hz gyroscope bandwidth, 1 kHz sample rate)
    config = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG, 1, &config, 1, HAL_MAX_DELAY);

    config = 0x00;  // ACCEL_HPF = 0x01 (5 Hz cutoff frequency)
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &config, 1, HAL_MAX_DELAY);

    // Set sample rate to 100 Hz
    config = 0x00; // 1000 / (1 + 9) = 100 Hz
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPRT_DIV, 1, &config, 1, HAL_MAX_DELAY);



    uint8_t whoAmI = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &whoAmI, sizeof(whoAmI), HAL_MAX_DELAY);
    if (whoAmI != 0x68) {
        printf("MPU6050 not detected. WHO_AM_I = 0x%02X\r\n", whoAmI);
    } else {
        printf("MPU6050 detected. WHO_AM_I = 0x%02X\r\n", whoAmI);
    }

    uint8_t fifoConfig, userCtrl, gConfig, sConfig;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_EN, 1, &fifoConfig, 1, HAL_MAX_DELAY);
//    printf("FIFO_EN: 0x%02X\r\n", fifoConfig);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, &userCtrl, 1, HAL_MAX_DELAY);
//    printf("USER_CTRL: 0x%02X\r\n", userCtrl);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &gConfig, 1, HAL_MAX_DELAY);
//    printf("ACCEL CONFIG: 0x%02X\r\n", gConfig);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &sConfig, 1, HAL_MAX_DELAY);
//    printf("GYRO CONFIG: 0x%02X\r\n", sConfig);
}

void MPU6050_ReadAll(MPU6050_Data *dataToProcess) {
    uint8_t buffer[14];

    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;

    // Read 14 bytes starting from register 0x3B
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 14, HAL_MAX_DELAY) == HAL_OK) {
        accelX = (int16_t)((buffer[0] << 8) | buffer[1]) - dataToProcess->accelOffsets[0];
        accelY = (int16_t)((buffer[2] << 8) | buffer[3]) - dataToProcess->accelOffsets[1];
        accelZ = (int16_t)((buffer[4] << 8) | buffer[5]) - dataToProcess->accelOffsets[2];
        gyroX  = (int16_t)((buffer[8] << 8) | buffer[9]) - dataToProcess->gyroOffsets[0];
        gyroY  = (int16_t)((buffer[10] << 8) | buffer[11]) - dataToProcess->gyroOffsets[1];
        gyroZ  = (int16_t)((buffer[12] << 8) | buffer[13]) - dataToProcess->gyroOffsets[2];
    } else {
    	//Error handle
    }

    dataToProcess->accelX = accelX / 16384.0f;
    dataToProcess->accelY = accelY / 16384.0f;
    dataToProcess->accelZ = accelZ / 16384.0f;

    dataToProcess->gyroX = gyroX / 131.0f;
    dataToProcess->gyroY = gyroY / 131.0f;
    dataToProcess->gyroZ = gyroZ / 131.0f;
}

void MPU6050_CalibrateInternal(MPU6050_Data *dataToProcess) {
    // Calculate offsets
    MPU6050_CalculateOffsets(dataToProcess->accelOffsets, dataToProcess->gyroOffsets);

    // Write offsets to MPU6050
    MPU6050_WriteOffsets(dataToProcess->accelOffsets, dataToProcess->gyroOffsets);

    // Optional: Print offsets for debugging
    printf("Accel Offsets: X=%d, Y=%d, Z=%d\r\n", dataToProcess->accelOffsets[0], dataToProcess->accelOffsets[1],
    		dataToProcess->accelOffsets[2]);
    printf("Gyro Offsets: X=%d, Y=%d, Z=%d\r\n", dataToProcess->gyroOffsets[0], dataToProcess->gyroOffsets[1],
    		dataToProcess->gyroOffsets[2]);

    HAL_Delay(2000);
}

void MPU6050_CalibrateExternal(MPU6050_Data *dataToProcess) {
    // Calculate offsets
    MPU6050_CalculateOffsets(dataToProcess->accelOffsets, dataToProcess->gyroOffsets);

    // Optional: Print offsets for debugging
    printf("Accel Offsets: X=%d, Y=%d, Z=%d\r\n", dataToProcess->accelOffsets[0], dataToProcess->accelOffsets[1],
    		dataToProcess->accelOffsets[2]);
    printf("Gyro Offsets: X=%d, Y=%d, Z=%d\r\n", dataToProcess->gyroOffsets[0], dataToProcess->gyroOffsets[1],
    		dataToProcess->gyroOffsets[2]);

    HAL_Delay(2000);
}

void MPU6050_CalculateOffsets(int16_t *accelOffsets, int16_t *gyroOffsets) {
    int32_t accelX_sum = 0, accelY_sum = 0, accelZ_sum = 0;
    int32_t gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;

    int n = 4000;

    uint8_t rawData[14];

    for (int i = 0; i < n; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 14, HAL_MAX_DELAY) == HAL_OK) {
        	accelX_sum += (int16_t)((rawData[0] << 8) | rawData[1]);
        	accelY_sum += (int16_t)((rawData[2] << 8) | rawData[3]);
        	accelZ_sum += (int16_t)((rawData[4] << 8) | rawData[5]) - 16384;
        	gyroX_sum  += (int16_t)((rawData[8] << 8) | rawData[9]);
        	gyroY_sum  += (int16_t)((rawData[10] << 8) | rawData[11]);
        	gyroZ_sum  += (int16_t)((rawData[12] << 8) | rawData[13]);
        } else {
        	//Error handle
        }
    }

    // Calculate average offsets
    accelOffsets[0] = (accelX_sum / n);
    accelOffsets[1] = (accelY_sum / n);
    accelOffsets[2] = (accelZ_sum / n);

    gyroOffsets[0] = (gyroX_sum / n);
    gyroOffsets[1] = (gyroY_sum / n);
    gyroOffsets[2] = (gyroZ_sum / n);
}

void MPU6050_WriteOffsets(int16_t *accelOffsets, int16_t *gyroOffsets) {
	uint8_t data[2];
    uint8_t readBack[2];

    printf("Accel Offsets: X=%d, Y=%d, Z=%d\r\n", accelOffsets[0], accelOffsets[1], accelOffsets[2]);
    printf("Gyro Offsets: X=%d, Y=%d, Z=%d\r\n", gyroOffsets[0], gyroOffsets[1], gyroOffsets[2]);

    // Write accelerometer offsets
    data[0] = (uint8_t)((accelOffsets[0] >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(accelOffsets[0] & 0xFE);        // Low byte
    printf("High Byte: 0x%02X, Low Byte: 0x%02X\r\n", data[0], data[1]);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XA_OFFS_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XA_OFFS_H, 1, readBack, 2, HAL_MAX_DELAY);
    int16_t writtenOffset = (int16_t)((readBack[0] << 8) | readBack[1]);
    printf("Accel X Offset Written: %d\r\n", writtenOffset);

    data[0] = (uint8_t)((accelOffsets[1] >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(accelOffsets[1] & 0xFE);        // Low byte
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YA_OFFS_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YA_OFFS_H, 1, readBack, 2, HAL_MAX_DELAY);
    writtenOffset = (int16_t)((readBack[0] << 8) | readBack[1]);
    printf("Accel Y Offset Written: %d\r\n", writtenOffset);

    data[0] = (uint8_t)((accelOffsets[2] >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(accelOffsets[2] & 0xFE);        // Low byte
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZA_OFFS_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZA_OFFS_H, 1, readBack, 2, HAL_MAX_DELAY);
    writtenOffset = (int16_t)((readBack[0] << 8) | readBack[1]);
    printf("Accel Z Offset Written: %d\r\n", writtenOffset);

    // Write gyroscope offsets
    data[0] = (uint8_t)((gyroOffsets[0] >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(gyroOffsets[0] & 0xFE);        // Low byte
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XG_OFFS_USRH, 1, readBack, 2, HAL_MAX_DELAY);
    writtenOffset = (int16_t)((readBack[0] << 8) | readBack[1]);
    printf("Gyro X Offset Written: %d\r\n", writtenOffset);

    data[0] = (uint8_t)((gyroOffsets[1] >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(gyroOffsets[1] & 0xFE);        // Low byte
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YG_OFFS_USRH, 1, readBack, 2, HAL_MAX_DELAY);
    writtenOffset = (int16_t)((readBack[0] << 8) | readBack[1]);
    printf("Gyro Y Offset Written: %d\r\n", writtenOffset);

    data[0] = (uint8_t)((gyroOffsets[2] >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(gyroOffsets[2] & 0xFE);        // Low byte
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT	, data, 2, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZG_OFFS_USRH, 1, readBack, 2, HAL_MAX_DELAY);
    writtenOffset = (int16_t)((readBack[0] << 8) | readBack[1]);
    printf("Gyro Z Offset Written: %d\r\n", writtenOffset);
}

void MPU6050_SoftReset(void) {
    uint8_t data_rst = 0x80; // DEVICE_RESET bit
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data_rst, sizeof(data_rst), HAL_MAX_DELAY);
    HAL_Delay(1000); // Allow time for the reset to complete
    uint8_t data[2] = {0x00, 0x00}; // Zero offset

    // Reset X-axis offset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XA_OFFS_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    // Reset Y-axis offset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YA_OFFS_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    // Reset Z-axis offset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZA_OFFS_H, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    // Reset X-axis offset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    // Reset Y-axis offset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    // Reset Z-axis offset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

}
