/*
 * imu.h
 *
 *  MPU9250 (accelerometer + gyroscope) and its onboard AK8963 magnetometer.
 *
 *  Created on: Jun 19, 2025
 *      Author: aaalb
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

/* I2C addresses, 7-bit values left-shifted by 1 for the STM32 HAL. */
#define MPU9250_ADDR   (0x68 << 1)
#define MAG_ADDR       (0x0C << 1)

/* Expected WHO_AM_I responses. The MPU9250 reports 0x71; a genuine MPU9255
 * reports 0x73 and the widely sold clones report 0x68. All three behave
 * identically for what we use, so accept any of them rather than refusing to
 * start on a board that would have worked fine. */
#define MPU9250_WHOAMI_MPU9250  0x71
#define MPU9250_WHOAMI_MPU9255  0x73
#define MPU9250_WHOAMI_CLONE    0x68
#define AK8963_WHOAMI           0x48

/* MPU9250 registers */
#define SMPLRT_DIV     0x19
#define MPU_CONFIG     0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG2  0x1D
#define INT_PIN_CFG    0x37
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define WHO_AM_I_REG   0x75

/* AK8963 registers (reachable only while the MPU9250 is in I2C bypass) */
#define AK8963_WIA     0x00
#define AK8963_ST1     0x02
#define AK8963_HXL     0x03
#define AK8963_ST2     0x09
#define AK8963_CNTL1   0x0A
#define AK8963_CNTL2   0x0B
#define AK8963_ASAX    0x10

#define AK8963_MODE_POWERDOWN   0x00
#define AK8963_MODE_FUSE_ROM    0x0F
#define AK8963_MODE_CONT2_16BIT 0x16  /* 100 Hz continuous, 16-bit output */
#define AK8963_ST1_DRDY         0x01
#define AK8963_ST2_HOFL         0x08  /* magnetic sensor overflow */

/* Telemetry health flags. Keep in sync with FLAG_* in
 * rpi_scripts/droneviz/telemetry.py -- the host decodes these by value. */
#define TLM_FLAG_IMU_ERROR     (1u << 0)  /* accel/gyro read failed; values are stale */
#define TLM_FLAG_MAG_STALE     (1u << 1)  /* no new magnetometer sample this cycle */
#define TLM_FLAG_MAG_OVERFLOW  (1u << 2)  /* AK8963 HOFL set; sample discarded */
#define TLM_FLAG_LOOP_OVERRUN  (1u << 3)  /* control loop missed its deadline */
#define TLM_FLAG_UNCALIBRATED  (1u << 4)  /* gyro bias calibration has not run */

/* Number of stationary samples averaged for the gyro bias estimate. At the
 * 1 kHz internal sample rate this takes about a second. */
#define IMU_GYRO_CAL_SAMPLES   1000

typedef enum {
	IMU_OK = 0,
	IMU_ERR_I2C,        /* a transfer failed outright -- check wiring and pull-ups */
	IMU_ERR_WHOAMI,     /* something answered, but it is not an MPU9250 */
	IMU_ERR_MAG_WHOAMI  /* accel/gyro fine, magnetometer did not answer */
} IMU_Status;

/*
 * Sensor readings and calibration state.
 *
 * Axes are the MPU9250's own frame (as silkscreened on the breakout), with the
 * magnetometer already rotated into it -- see IMU_ReadMag. Whatever rotation
 * maps that onto the airframe is a separate concern for the flight code, not
 * this driver.
 */
typedef struct {
	float ax, ay, az;   /* specific force, m/s^2 */
	float gx, gy, gz;   /* angular rate, deg/s, bias-corrected */
	float mx, my, mz;   /* magnetic field, uT */
	float temp_c;       /* die temperature, degrees C */

	float accel_sens;   /* LSB per g */
	float gyro_sens;    /* LSB per deg/s */
	float mag_adj[3];   /* AK8963 factory sensitivity adjustment, per axis */
	float gyro_bias[3]; /* deg/s, subtracted on every read */

	uint32_t i2c_errors;        /* cumulative failed transfers */
	uint8_t consecutive_errors; /* reset by any successful transfer */
	uint8_t flags;              /* TLM_FLAG_* for the most recent read */
	uint8_t calibrated;         /* non-zero once IMU_CalibrateGyro has run */
} MPU9250_Data;

/* Full bring-up: reset, clock source, ranges, filters, and the magnetometer.
 * Leaves the gyro bias at zero -- call IMU_CalibrateGyro once the vehicle is
 * sitting still. */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);

/* Average the gyro while stationary and store the result as bias. Blocks for
 * roughly IMU_GYRO_CAL_SAMPLES milliseconds. Any motion during this call is
 * baked in as error, so hold still. */
IMU_Status IMU_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);

/* One accel + temperature + gyro burst plus a magnetometer read. Updates
 * data->flags. Returns IMU_ERR_I2C if the accel/gyro burst failed, in which
 * case the previous values are retained. */
IMU_Status IMU_ReadAll(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);

/* Individual reads, kept for bring-up and debugging. IMU_ReadAll is what the
 * control loop should use: it takes accel and gyro in a single transaction, so
 * they describe the same instant. */
IMU_Status IMU_ReadAccel(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);
IMU_Status IMU_ReadGyro(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);
IMU_Status IMU_ReadMag(I2C_HandleTypeDef *hi2c, MPU9250_Data *data);

#endif /* INC_IMU_H_ */
