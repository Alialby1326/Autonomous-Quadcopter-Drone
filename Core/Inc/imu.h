/*
 * imu.h
 *
 *  MPU9250 (accelerometer + gyroscope) and its onboard AK8963 magnetometer,
 *  over SPI2.
 *
 *      SCK  PB13 -> SCL/SCLK      MOSI PB15 -> SDA/SDI
 *      MISO PB14 -> AD0/SDO       CS   PB12 -> NCS
 *
 *  The AK8963 is not on SPI at all: it hangs off the MPU9250's internal I2C
 *  bus. The MPU9250's own I2C master polls it and copies the result into
 *  EXT_SENS_DATA, so one SPI burst returns accel, temperature, gyro and mag
 *  together.
 *
 *  Hand-written: this module configures SPI2 and its pins itself. SPI2 and
 *  PB12-PB15 must stay unused in CubeMX.
 *
 *  Created on: Jun 19, 2025
 *      Author: aaalb
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

/* AK8963 address on the MPU9250's internal I2C bus (7-bit). */
#define AK8963_I2C_ADDR 0x0C

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
#define I2C_MST_CTRL   0x24
#define I2C_SLV0_ADDR  0x25
#define I2C_SLV0_REG   0x26
#define I2C_SLV0_CTRL  0x27
#define I2C_SLV4_ADDR  0x31
#define I2C_SLV4_REG   0x32
#define I2C_SLV4_DO    0x33
#define I2C_SLV4_CTRL  0x34
#define I2C_SLV4_DI    0x35
#define I2C_MST_STATUS 0x36
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43
#define EXT_SENS_DATA_00 0x49
#define I2C_MST_DELAY_CTRL 0x67
#define USER_CTRL      0x6A
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define WHO_AM_I_REG   0x75

#define USER_CTRL_I2C_MST_EN    0x20
#define USER_CTRL_I2C_IF_DIS    0x10  /* SPI only; stops a glitch flipping it to I2C */
#define I2C_SLV_EN              0x80
#define I2C_SLV_READ            0x80  /* OR'd into a slave address */
#define I2C_MST_STATUS_SLV4_DONE 0x40
#define I2C_MST_STATUS_SLV4_NACK 0x10

/* AK8963 registers, reached through the MPU9250's I2C master */
#define AK8963_WIA     0x00
#define AK8963_ST1     0x02
#define AK8963_HXL     0x03
#define AK8963_ST2     0x09
#define AK8963_CNTL1   0x0A
#define AK8963_CNTL2   0x0B
#define AK8963_ASAX    0x10

#define AK8963_CNTL2_SRST       0x01

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
	IMU_ERR_BUS,        /* an SPI transfer failed outright */
	IMU_ERR_WHOAMI,     /* wrong or no answer -- not an MPU9250, or check wiring */
	IMU_ERR_MAG_WHOAMI  /* accel/gyro fine, magnetometer did not answer */
} IMU_Status;

/*
 * Sensor readings and calibration state.
 *
 * Axes are the MPU9250's own frame (as silkscreened on the breakout), with the
 * magnetometer already rotated into it -- see decode_mag in imu.c. Whatever rotation
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

	uint32_t bus_errors;        /* cumulative failed transfers */
	uint8_t consecutive_errors; /* reset by any successful transfer */
	uint8_t flags;              /* TLM_FLAG_* for the most recent read */
	uint8_t calibrated;         /* non-zero once IMU_CalibrateGyro has run */
} MPU9250_Data;

/* Full bring-up: SPI2 and its pins, then reset, clock source, ranges, filters,
 * and the magnetometer. Leaves the gyro bias at zero -- call
 * IMU_CalibrateGyro once the vehicle is sitting still. */
IMU_Status IMU_Init(MPU9250_Data *data);

/* Average the gyro while stationary and store the result as bias. Blocks for
 * roughly IMU_GYRO_CAL_SAMPLES milliseconds. Any motion during this call is
 * baked in as error, so hold still. */
IMU_Status IMU_CalibrateGyro(MPU9250_Data *data);

/* One SPI burst: accel, temperature, gyro and the latest magnetometer sample,
 * all from the same instant. Updates data->flags. Returns IMU_ERR_BUS if the
 * transfer failed, in which case the previous values are retained. */
IMU_Status IMU_ReadAll(MPU9250_Data *data);

#endif /* INC_IMU_H_ */
