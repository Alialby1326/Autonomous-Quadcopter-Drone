/*
 * imu.c
 *
 *  Created on: Jun 19, 2025
 *      Author: aaalb
 */

#include "imu.h"

uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
    uint8_t check;
    uint8_t pwr_data;
    uint8_t config;

    // Read WHO_AM_I to detect sensor
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, WHO_AM_I_REG, 1, &check, 1, 50);
    if (check != 0x71) return 0; // failure

    // Turn on sensor
    pwr_data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, PWR_MGMT_1, 1, &pwr_data, 1, 50);
    HAL_Delay(10);

    // Initialize Accelerometer Sensitivity
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, ACCEL_CONFIG, 1, &config, 1, 50);
    switch ((config >> 3) & 0x03){
    case 0: data->accel_sens = 16384.0f; break;
    case 1: data->accel_sens = 8192.0f; break;
    case 2: data->accel_sens = 4096.0f; break;
    case 3: data->accel_sens = 2048.0f; break;
    }

    // Initialize Gyroscope Sensitivity
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, GYRO_CONFIG, 1, &config, 1, 50);
    switch ((config >> 3) & 0x03){
    case 0: data->gyro_sens = 131.0f; break;
    case 1: data->gyro_sens = 65.5f; break;
    case 2: data->gyro_sens = 32.8f; break;
    case 3: data->gyro_sens = 16.4f; break;
    }

    return 1; // success
}

void IMU_MagINIT(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t bypass_en = 0x02; //1 bit shift
	uint8_t mode16bit = 0x16;  //to enable 16 bit output
	uint8_t pdown = 0x00;

	//enter bypass mode and initialize 16 bit output
	MPU9250_Init(hi2c, data);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, INT_PIN_CFG, 1, &bypass_en, 1, 50);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(hi2c, MAG_ADDR, CNTL1, 1, &pdown, 1, 50);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(hi2c, MAG_ADDR, CNTL1, 1, &mode16bit, 1, 50);
	HAL_Delay(10);

}

void IMU_ReadAccel(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
    uint8_t raw[6];

    //read then convert
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, ACCEL_XOUT_H, 1, raw, 6, 50);

    data->ax = (((int16_t)(raw[0] << 8 | raw[1]) / data->accel_sens) * 9.81f);
    data->ay = (((int16_t)(raw[2] << 8 | raw[3]) / data->accel_sens) * 9.81f);
    data->az = (((int16_t)(raw[4] << 8 | raw[5]) / data->accel_sens) * 9.81f);
}

void IMU_ReadGyro(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t graw[6];

    //read then convert
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, GYRO_XOUT_H, 1, graw, 6, 50);

	data->gx = ((int16_t)(graw[0] << 8 | graw[1]) / data->gyro_sens);
	data->gy = ((int16_t)(graw[2] << 8 | graw[3]) / data->gyro_sens);
	data->gz = ((int16_t)(graw[4] << 8 | graw[5]) / data->gyro_sens);
}

void IMU_ReadMag(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t mraw[6];
	uint8_t mcheck;
	uint8_t mcheck2;

	//status 1
	HAL_I2C_Mem_Read(hi2c, MAG_ADDR, 0x02, 1, &mcheck, 1, 50);
	if (!(mcheck & 0x01)){
		return;
	}

	//read data
	HAL_I2C_Mem_Read(hi2c, MAG_ADDR, MAG_XOUT_L, 1, mraw, 6, 50);

	//status 2
	HAL_I2C_Mem_Read(hi2c, MAG_ADDR, 0x09, 1, &mcheck2, 1, 50);

	//convert 0.15 uT per LSB
	data->mx = ((int16_t)(mraw[1] << 8 | mraw[0]) * 0.15f);
	data->my = ((int16_t)(mraw[3] << 8 | mraw[2]) * 0.15f);
	data->mz = ((int16_t)(mraw[5] << 8 | mraw[4]) * 0.15f);
}
