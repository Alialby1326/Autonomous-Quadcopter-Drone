/*
 * imu.c
 *
 *  Created on: Jun 19, 2025
 *      Author: aaalb
 */

#include "imu.h"

#define I2C_TIMEOUT_MS 50

/* Full-scale ranges. +-8 g and +-1000 deg/s leave headroom for a quad's
 * attitude maneuvers without throwing away resolution the way the maximum
 * ranges would. */
#define ACCEL_FS_SEL   2      /* 0:+-2g  1:+-4g  2:+-8g  3:+-16g */
#define GYRO_FS_SEL    2      /* 0:+-250 1:+-500 2:+-1000 3:+-2000 deg/s */

#define GRAVITY_MS2    9.80665f

/* AK8963 16-bit mode: 0.15 uT per LSB. */
#define MAG_LSB_UT     0.15f

static const float ACCEL_SENS_LSB_PER_G[4]   = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float GYRO_SENS_LSB_PER_DPS[4]  = {131.0f, 65.5f, 32.8f, 16.4f};

/*
 * Every I2C access goes through these two so that failures are counted rather
 * than ignored. The old driver discarded every HAL status, which meant a
 * disconnected sensor produced a stream of plausible-looking frozen values
 * instead of an error -- the single most dangerous failure mode for a flight
 * controller, because nothing downstream can tell it apart from level flight.
 */
static HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, MPU9250_Data *data,
                                  uint16_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, dev, reg, 1, buf, len, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		data->i2c_errors++;
		if (data->consecutive_errors < 255) {
			data->consecutive_errors++;
		}
	} else {
		data->consecutive_errors = 0;
	}
	return status;
}

static HAL_StatusTypeDef imu_write(I2C_HandleTypeDef *hi2c, MPU9250_Data *data,
                                   uint16_t dev, uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, dev, reg, 1, &value, 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		data->i2c_errors++;
		if (data->consecutive_errors < 255) {
			data->consecutive_errors++;
		}
	} else {
		data->consecutive_errors = 0;
	}
	return status;
}

/* Assemble a big-endian signed 16-bit sample. The accel and gyro are
 * big-endian; the magnetometer, unusually, is little-endian. */
static inline int16_t be16(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static inline int16_t le16(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static IMU_Status mag_init(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t who = 0;
	uint8_t asa[3] = {128, 128, 128};

	/* The AK8963 sits on its own I2C bus behind the MPU9250. Bypass mode wires
	 * that bus through to the host so we can address it directly. */
	if (imu_write(hi2c, data, MPU9250_ADDR, INT_PIN_CFG, 0x02) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(10);

	if (imu_read(hi2c, data, MAG_ADDR, AK8963_WIA, &who, 1) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	if (who != AK8963_WHOAMI) {
		return IMU_ERR_MAG_WHOAMI;
	}

	/* Mode changes must pass through power-down; the datasheet requires it and
	 * skipping it leaves the part in an undefined state. */
	if (imu_write(hi2c, data, MAG_ADDR, AK8963_CNTL1, AK8963_MODE_POWERDOWN) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(10);

	/*
	 * Read the factory sensitivity adjustment burned into fuse ROM. Each axis
	 * of an AK8963 has a slightly different gain, measured at the factory and
	 * stored here; without applying it the field magnitude changes as the
	 * board rotates, which reads as heading error. This is the calibration the
	 * project README lists as outstanding -- distinct from hard-iron
	 * calibration, which corrects for the magnetic environment rather than the
	 * part itself and has to be redone whenever the surrounding hardware
	 * changes.
	 */
	if (imu_write(hi2c, data, MAG_ADDR, AK8963_CNTL1, AK8963_MODE_FUSE_ROM) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(10);
	if (imu_read(hi2c, data, MAG_ADDR, AK8963_ASAX, asa, 3) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	for (int i = 0; i < 3; i++) {
		data->mag_adj[i] = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;
	}

	if (imu_write(hi2c, data, MAG_ADDR, AK8963_CNTL1, AK8963_MODE_POWERDOWN) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(10);
	if (imu_write(hi2c, data, MAG_ADDR, AK8963_CNTL1, AK8963_MODE_CONT2_16BIT) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(10);

	return IMU_OK;
}

IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t who = 0;

	for (int i = 0; i < 3; i++) {
		data->mag_adj[i] = 1.0f;
		data->gyro_bias[i] = 0.0f;
	}
	data->i2c_errors = 0;
	data->consecutive_errors = 0;
	data->flags = TLM_FLAG_UNCALIBRATED;
	data->calibrated = 0;
	data->mx = data->my = data->mz = 0.0f;

	if (imu_read(hi2c, data, MPU9250_ADDR, WHO_AM_I_REG, &who, 1) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	if (who != MPU9250_WHOAMI_MPU9250 && who != MPU9250_WHOAMI_MPU9255 &&
	    who != MPU9250_WHOAMI_CLONE) {
		return IMU_ERR_WHOAMI;
	}

	/* Reset, then wait out the startup time before touching anything else. */
	if (imu_write(hi2c, data, MPU9250_ADDR, PWR_MGMT_1, 0x80) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(100);

	/*
	 * Clock source 1 = PLL referenced to the X gyro. The reset default of 0
	 * uses the internal 20 MHz RC oscillator, whose frequency wanders with
	 * temperature -- so the sample interval wanders too, and the gyro
	 * integration inherits a scale error that looks exactly like drift.
	 */
	if (imu_write(hi2c, data, MPU9250_ADDR, PWR_MGMT_1, 0x01) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	HAL_Delay(10);
	if (imu_write(hi2c, data, MPU9250_ADDR, PWR_MGMT_2, 0x00) != HAL_OK) {
		return IMU_ERR_I2C;
	}

	/*
	 * DLPF_CFG 3 gives a 41 Hz gyro bandwidth at a 1 kHz internal rate, and
	 * SMPLRT_DIV 0 keeps the output there. 41 Hz sits above the attitude
	 * dynamics we care about and well below the propeller vibration that will
	 * dominate this signal once motors are turning.
	 */
	if (imu_write(hi2c, data, MPU9250_ADDR, MPU_CONFIG, 0x03) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	if (imu_write(hi2c, data, MPU9250_ADDR, SMPLRT_DIV, 0x00) != HAL_OK) {
		return IMU_ERR_I2C;
	}

	/*
	 * Write the ranges rather than reading back whatever happened to be in
	 * the registers. The low two bits of GYRO_CONFIG are FCHOICE_B, left at
	 * zero so the DLPF configured above is actually used -- setting them
	 * bypasses it and returns the raw 32 kHz signal.
	 */
	if (imu_write(hi2c, data, MPU9250_ADDR, GYRO_CONFIG, (uint8_t)(GYRO_FS_SEL << 3)) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	if (imu_write(hi2c, data, MPU9250_ADDR, ACCEL_CONFIG, (uint8_t)(ACCEL_FS_SEL << 3)) != HAL_OK) {
		return IMU_ERR_I2C;
	}
	/* Accelerometer DLPF, 41 Hz, matching the gyro. */
	if (imu_write(hi2c, data, MPU9250_ADDR, ACCEL_CONFIG2, 0x03) != HAL_OK) {
		return IMU_ERR_I2C;
	}

	data->accel_sens = ACCEL_SENS_LSB_PER_G[ACCEL_FS_SEL];
	data->gyro_sens = GYRO_SENS_LSB_PER_DPS[GYRO_FS_SEL];

	HAL_Delay(10);

	return mag_init(hi2c, data);
}

IMU_Status IMU_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t raw[6];
	float sum[3] = {0.0f, 0.0f, 0.0f};
	uint32_t taken = 0;

	for (int i = 0; i < 3; i++) {
		data->gyro_bias[i] = 0.0f;
	}

	for (uint32_t i = 0; i < IMU_GYRO_CAL_SAMPLES; i++) {
		if (imu_read(hi2c, data, MPU9250_ADDR, GYRO_XOUT_H, raw, 6) == HAL_OK) {
			sum[0] += (float)be16(&raw[0]);
			sum[1] += (float)be16(&raw[2]);
			sum[2] += (float)be16(&raw[4]);
			taken++;
		}
		HAL_Delay(1);
	}

	/* Refuse to record a bias from a handful of samples: a bad average here
	 * would be subtracted from every future reading. */
	if (taken < IMU_GYRO_CAL_SAMPLES / 2) {
		return IMU_ERR_I2C;
	}

	for (int i = 0; i < 3; i++) {
		data->gyro_bias[i] = (sum[i] / (float)taken) / data->gyro_sens;
	}
	data->calibrated = 1;
	data->flags &= (uint8_t)~TLM_FLAG_UNCALIBRATED;

	return IMU_OK;
}

IMU_Status IMU_ReadAccel(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t raw[6];

	if (imu_read(hi2c, data, MPU9250_ADDR, ACCEL_XOUT_H, raw, 6) != HAL_OK) {
		return IMU_ERR_I2C;
	}

	data->ax = ((float)be16(&raw[0]) / data->accel_sens) * GRAVITY_MS2;
	data->ay = ((float)be16(&raw[2]) / data->accel_sens) * GRAVITY_MS2;
	data->az = ((float)be16(&raw[4]) / data->accel_sens) * GRAVITY_MS2;

	return IMU_OK;
}

IMU_Status IMU_ReadGyro(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t raw[6];

	if (imu_read(hi2c, data, MPU9250_ADDR, GYRO_XOUT_H, raw, 6) != HAL_OK) {
		return IMU_ERR_I2C;
	}

	data->gx = (float)be16(&raw[0]) / data->gyro_sens - data->gyro_bias[0];
	data->gy = (float)be16(&raw[2]) / data->gyro_sens - data->gyro_bias[1];
	data->gz = (float)be16(&raw[4]) / data->gyro_sens - data->gyro_bias[2];

	return IMU_OK;
}

IMU_Status IMU_ReadMag(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	uint8_t st1 = 0;
	uint8_t raw[7];

	data->flags &= (uint8_t)~(TLM_FLAG_MAG_STALE | TLM_FLAG_MAG_OVERFLOW);

	if (imu_read(hi2c, data, MAG_ADDR, AK8963_ST1, &st1, 1) != HAL_OK) {
		data->flags |= TLM_FLAG_MAG_STALE;
		return IMU_ERR_I2C;
	}
	if (!(st1 & AK8963_ST1_DRDY)) {
		/* The magnetometer runs at 100 Hz while the control loop runs faster,
		 * so this is routine, not a fault. The previous sample stays in place
		 * and the flag tells the host it is not fresh. */
		data->flags |= TLM_FLAG_MAG_STALE;
		return IMU_OK;
	}

	/* HXL..ST2 are contiguous, so one burst gets the data and the status byte.
	 * ST2 must be read to release the measurement latch -- omit it and the
	 * AK8963 stops producing new samples. */
	if (imu_read(hi2c, data, MAG_ADDR, AK8963_HXL, raw, 7) != HAL_OK) {
		data->flags |= TLM_FLAG_MAG_STALE;
		return IMU_ERR_I2C;
	}

	if (raw[6] & AK8963_ST2_HOFL) {
		/* Overflow means the field exceeded the sensor's range, usually from
		 * something magnetic nearby. The reading is not merely noisy, it is
		 * meaningless, so discard it rather than filtering it. */
		data->flags |= TLM_FLAG_MAG_OVERFLOW | TLM_FLAG_MAG_STALE;
		return IMU_OK;
	}

	{
		float mx = (float)le16(&raw[0]) * MAG_LSB_UT * data->mag_adj[0];
		float my = (float)le16(&raw[2]) * MAG_LSB_UT * data->mag_adj[1];
		float mz = (float)le16(&raw[4]) * MAG_LSB_UT * data->mag_adj[2];

		/*
		 * The AK8963 die is bonded into the package rotated relative to the
		 * accelerometer and gyroscope, so its axes are not theirs:
		 *
		 *     mag X -> accel Y,  mag Y -> accel X,  mag Z -> -accel Z
		 *
		 * Remapping here means everything downstream sees one consistent
		 * frame. Skipping it produces a heading that responds smoothly to
		 * rotation and is simply wrong -- the kind of error that survives a
		 * casual check because nothing about it looks broken.
		 */
		data->mx = my;
		data->my = mx;
		data->mz = -mz;
	}

	return IMU_OK;
}

IMU_Status IMU_ReadAll(I2C_HandleTypeDef *hi2c, MPU9250_Data *data)
{
	/* ACCEL_XOUT_H .. GYRO_ZOUT_L is one contiguous 14-byte block: six accel
	 * bytes, two temperature, six gyro. Reading it in a single transaction is
	 * both faster than three separate reads and, more importantly, gives accel
	 * and gyro values captured at the same instant -- which is what the filter
	 * assumes when it fuses them. */
	uint8_t raw[14];

	if (imu_read(hi2c, data, MPU9250_ADDR, ACCEL_XOUT_H, raw, 14) != HAL_OK) {
		data->flags |= TLM_FLAG_IMU_ERROR;
		return IMU_ERR_I2C;
	}
	data->flags &= (uint8_t)~TLM_FLAG_IMU_ERROR;

	data->ax = ((float)be16(&raw[0]) / data->accel_sens) * GRAVITY_MS2;
	data->ay = ((float)be16(&raw[2]) / data->accel_sens) * GRAVITY_MS2;
	data->az = ((float)be16(&raw[4]) / data->accel_sens) * GRAVITY_MS2;

	/* Datasheet scaling for the on-die temperature sensor. */
	data->temp_c = (float)be16(&raw[6]) / 333.87f + 21.0f;

	data->gx = (float)be16(&raw[8]) / data->gyro_sens - data->gyro_bias[0];
	data->gy = (float)be16(&raw[10]) / data->gyro_sens - data->gyro_bias[1];
	data->gz = (float)be16(&raw[12]) / data->gyro_sens - data->gyro_bias[2];

	IMU_ReadMag(hi2c, data);

	return IMU_OK;
}
