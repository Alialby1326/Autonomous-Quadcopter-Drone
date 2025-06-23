/*
 * imu.h
 *
 *  Created on: Jun 19, 2025
 *      Author: aaalb
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

//default I2C address (7-bit, left-shifted by 1 for STM HAL)
#define MPU9250_ADDR   (0x68 << 1)

//addresses
#define WHO_AM_I_REG   0x75
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define GYRO_XOUT_H    0x43
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define INT_PIN_CFG    0x37
#define CNTL1          0x0A
#define MAG_ADDR       (0x0C << 1)
#define MAG_XOUT_L     0x03

//IMU data
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    float accel_sens, gyro_sens;
} MPU9250_Data;



//functions
uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c);
void IMU_ReadAccel(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);
void IMU_ReadGyro(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);
void IMU_ReadMag(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);
void IMU_MagINIT(I2C_HandleTypeDef *hi2c);

#endif /* INC_IMU_H_ */
