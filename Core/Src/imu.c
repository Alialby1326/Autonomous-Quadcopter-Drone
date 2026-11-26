/*
 * imu.c
 *
 *  See imu.h for the wiring and why the magnetometer is read the way it is.
 *
 *  Created on: Jun 19, 2025
 *      Author: aaalb
 */

#include "imu.h"

#include <string.h>

#define IMU_CS_PORT    GPIOB
#define IMU_CS_PIN     GPIO_PIN_12
#define SPI_TIMEOUT_MS 5

/* MPU9250 SPI limits: 1 MHz for every register, 20 MHz for reading the sensor
 * and interrupt registers (INT_STATUS .. EXT_SENS_DATA_23). The fast limit is
 * set well under 20 MHz because the IMU is on jumper wires, not a PCB. */
#define SPI_SLOW_MAX_HZ   1000000u
#define SPI_FAST_MAX_HZ   8000000u
#define SENSOR_REG_FIRST  0x3A
#define SENSOR_REG_LAST   0x60

/* ACCEL_XOUT_H .. GYRO_ZOUT_L, then EXT_SENS_DATA_00..07 holding the AK8963's
 * ST1, HXL..HZH, ST2. The registers are contiguous, so this is one burst. */
#define MAG_EXT_LEN    8
#define IMU_BURST_LEN  (14 + MAG_EXT_LEN)
#define SPI_MAX_LEN    IMU_BURST_LEN

/* I2C_MST_CTRL clock select 13 = 400 kHz on the internal bus. */
#define I2C_MST_CLK_400KHZ 0x0D

/* The internal I2C master runs once per sample (1 kHz). Slave 0 skips 9 of
 * every 10, so it polls the AK8963 at 100 Hz -- its own output rate -- and
 * each poll usually lands exactly one new sample. */
#define MAG_POLL_DLY   9
#define I2C_MST_DELAY_ES_SHADOW 0x80  /* publish all 8 bytes together, never half a sample */
#define I2C_MST_SLV0_DLY_EN     0x01

#define MAG_SLV4_TIMEOUT_MS 20

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

static SPI_HandleTypeDef hspi;
static uint32_t spi_prescaler_slow;
static uint32_t spi_prescaler_fast;
static uint8_t spi_tx[SPI_MAX_LEN + 1];
static uint8_t spi_rx[SPI_MAX_LEN + 1];
static uint8_t mag_ready;

/* ------------------------------------------------------------------------- */
/* SPI2                                                                      */
/* ------------------------------------------------------------------------- */

/* The fastest divider that stays at or under max_hz. Derived from PCLK1
 * rather than hard-coded, so retuning the clock tree keeps the SPI inside the
 * MPU9250's limits. */
static uint32_t spi_prescaler_for(uint32_t max_hz)
{
	static const uint32_t prescalers[8] = {
		SPI_BAUDRATEPRESCALER_2,  SPI_BAUDRATEPRESCALER_4,
		SPI_BAUDRATEPRESCALER_8,  SPI_BAUDRATEPRESCALER_16,
		SPI_BAUDRATEPRESCALER_32, SPI_BAUDRATEPRESCALER_64,
		SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
	};
	uint32_t pclk = HAL_RCC_GetPCLK1Freq();  /* SPI2 is on APB1 */

	for (uint32_t i = 0u; i < 8u; i++) {
		if ((pclk >> (i + 1u)) <= max_hz) {
			return prescalers[i];
		}
	}
	return SPI_BAUDRATEPRESCALER_256;
}

static void spi_set_prescaler(uint32_t prescaler)
{
	if (hspi.Init.BaudRatePrescaler == prescaler) {
		return;
	}
	/* BR may only change while the peripheral is disabled. HAL re-enables it
	 * on the next transfer. CS is high here, so the SCK change is unseen. */
	__HAL_SPI_DISABLE(&hspi);
	MODIFY_REG(hspi.Instance->CR1, SPI_CR1_BR, prescaler);
	hspi.Init.BaudRatePrescaler = prescaler;
}

static HAL_StatusTypeDef spi_init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* CS high before the pin becomes an output, so the MPU9250 never sees a
	 * stray select during bring-up. */
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
	gpio.Pin = IMU_CS_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(IMU_CS_PORT, &gpio);

	gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &gpio);

	spi_prescaler_slow = spi_prescaler_for(SPI_SLOW_MAX_HZ);
	spi_prescaler_fast = spi_prescaler_for(SPI_FAST_MAX_HZ);

	hspi.Instance = SPI2;
	hspi.Init.Mode = SPI_MODE_MASTER;
	hspi.Init.Direction = SPI_DIRECTION_2LINES;
	hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;   /* mode 3 */
	hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi.Init.NSS = SPI_NSS_SOFT;
	hspi.Init.BaudRatePrescaler = spi_prescaler_slow;
	hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi.Init.CRCPolynomial = 10;
	return HAL_SPI_Init(&hspi);
}

static HAL_StatusTypeDef spi_transfer(uint16_t len)
{
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
	/* Returns only once BSY clears, so CS never rises mid-byte. */
	status = HAL_SPI_TransmitReceive(&hspi, spi_tx, spi_rx, len, SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
	return status;
}

/*
 * Every register access goes through these two so that failures are counted
 * rather than ignored. A dead sensor must show up as an error, never as a
 * stream of plausible frozen values: nothing downstream can tell those apart
 * from level flight.
 *
 * Note that SPI has no acknowledge, so a disconnected MPU9250 usually reads
 * back 0x00 or 0xFF rather than failing here. That is why IMU_Init checks
 * WHO_AM_I before anything else.
 */
static HAL_StatusTypeDef count_errors(MPU9250_Data *data, HAL_StatusTypeDef status)
{
	if (status != HAL_OK) {
		data->bus_errors++;
		if (data->consecutive_errors < 255) {
			data->consecutive_errors++;
		}
	} else {
		data->consecutive_errors = 0;
	}
	return status;
}

static HAL_StatusTypeDef imu_read(MPU9250_Data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef status;

	if (len == 0u || len > SPI_MAX_LEN) {
		return count_errors(data, HAL_ERROR);
	}

	if (reg >= SENSOR_REG_FIRST && (uint16_t)(reg + len - 1u) <= SENSOR_REG_LAST) {
		spi_set_prescaler(spi_prescaler_fast);
	} else {
		spi_set_prescaler(spi_prescaler_slow);
	}

	memset(spi_tx, 0, (size_t)len + 1u);
	spi_tx[0] = (uint8_t)(reg | 0x80u);   /* bit 7 set = read */
	status = spi_transfer((uint16_t)(len + 1u));
	if (status == HAL_OK) {
		memcpy(buf, &spi_rx[1], len);
	}
	return count_errors(data, status);
}

static HAL_StatusTypeDef imu_write(MPU9250_Data *data, uint8_t reg, uint8_t value)
{
	spi_set_prescaler(spi_prescaler_slow);   /* writes are 1 MHz-only */
	spi_tx[0] = (uint8_t)(reg & 0x7Fu);
	spi_tx[1] = value;
	return count_errors(data, spi_transfer(2u));
}

/* ------------------------------------------------------------------------- */
/* AK8963, through the MPU9250's I2C master                                  */
/* ------------------------------------------------------------------------- */

/*
 * One transaction on the internal bus using slave 4, which is single-shot:
 * it runs once and reports DONE or NACK in I2C_MST_STATUS. Slave 0 repeats
 * every sample, so using it for writes would hammer CNTL1 forever.
 *
 * Writing I2C_SLV4_CTRL also writes I2C_MST_DLY (its low bits) to zero; the
 * poll delay is set after the last slave 4 transaction, in mag_init.
 */
static IMU_Status mag_slv4(MPU9250_Data *data, uint8_t addr, uint8_t reg,
                           uint8_t out, uint8_t *in)
{
	uint8_t st = 0;
	uint32_t start;

	if (imu_write(data, I2C_SLV4_ADDR, addr) != HAL_OK ||
	    imu_write(data, I2C_SLV4_REG, reg) != HAL_OK ||
	    imu_write(data, I2C_SLV4_DO, out) != HAL_OK ||
	    imu_read(data, I2C_MST_STATUS, &st, 1) != HAL_OK ||   /* clears stale flags */
	    imu_write(data, I2C_SLV4_CTRL, I2C_SLV_EN) != HAL_OK) {
		return IMU_ERR_BUS;
	}

	start = HAL_GetTick();
	for (;;) {
		if (imu_read(data, I2C_MST_STATUS, &st, 1) != HAL_OK) {
			return IMU_ERR_BUS;
		}
		if (st & I2C_MST_STATUS_SLV4_NACK) {
			return IMU_ERR_MAG_WHOAMI;
		}
		if (st & I2C_MST_STATUS_SLV4_DONE) {
			break;
		}
		if (HAL_GetTick() - start > MAG_SLV4_TIMEOUT_MS) {
			return IMU_ERR_MAG_WHOAMI;
		}
	}

	if (in != NULL && imu_read(data, I2C_SLV4_DI, in, 1) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	return IMU_OK;
}

static IMU_Status mag_write(MPU9250_Data *data, uint8_t reg, uint8_t value)
{
	return mag_slv4(data, AK8963_I2C_ADDR, reg, value, NULL);
}

static IMU_Status mag_read(MPU9250_Data *data, uint8_t reg, uint8_t *value)
{
	return mag_slv4(data, AK8963_I2C_ADDR | I2C_SLV_READ, reg, 0, value);
}

static IMU_Status mag_init(MPU9250_Data *data)
{
	IMU_Status s;
	uint8_t who = 0;
	uint8_t asa[3] = {128, 128, 128};

	if (imu_write(data, USER_CTRL, USER_CTRL_I2C_IF_DIS | USER_CTRL_I2C_MST_EN) != HAL_OK ||
	    imu_write(data, I2C_MST_CTRL, I2C_MST_CLK_400KHZ) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	HAL_Delay(10);

	/* Resetting the MPU9250 does not reset the AK8963; do it explicitly so a
	 * warm reboot starts from the same state as a cold one. */
	if ((s = mag_write(data, AK8963_CNTL2, AK8963_CNTL2_SRST)) != IMU_OK) {
		return s;
	}
	HAL_Delay(10);

	if ((s = mag_read(data, AK8963_WIA, &who)) != IMU_OK) {
		return s;
	}
	if (who != AK8963_WHOAMI) {
		return IMU_ERR_MAG_WHOAMI;
	}

	/* Mode changes must pass through power-down; the datasheet requires it and
	 * skipping it leaves the part in an undefined state. */
	if ((s = mag_write(data, AK8963_CNTL1, AK8963_MODE_POWERDOWN)) != IMU_OK) {
		return s;
	}
	HAL_Delay(10);

	/*
	 * Read the factory sensitivity adjustment burned into fuse ROM. Each axis
	 * of an AK8963 has a slightly different gain, measured at the factory and
	 * stored here; without applying it the field magnitude changes as the
	 * board rotates, which reads as heading error. Distinct from hard-iron
	 * calibration, which corrects for the magnetic environment rather than the
	 * part itself and has to be redone whenever the surrounding hardware
	 * changes.
	 */
	if ((s = mag_write(data, AK8963_CNTL1, AK8963_MODE_FUSE_ROM)) != IMU_OK) {
		return s;
	}
	HAL_Delay(10);
	for (uint8_t i = 0; i < 3u; i++) {
		if ((s = mag_read(data, (uint8_t)(AK8963_ASAX + i), &asa[i])) != IMU_OK) {
			return s;
		}
	}
	for (int i = 0; i < 3; i++) {
		data->mag_adj[i] = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;
	}

	if ((s = mag_write(data, AK8963_CNTL1, AK8963_MODE_POWERDOWN)) != IMU_OK) {
		return s;
	}
	HAL_Delay(10);
	if ((s = mag_write(data, AK8963_CNTL1, AK8963_MODE_CONT2_16BIT)) != IMU_OK) {
		return s;
	}
	HAL_Delay(10);

	/* Hand the bus to slave 0: read ST1..ST2 into EXT_SENS_DATA_00..07 every
	 * MAG_POLL_DLY + 1 samples. Reading ST2 each time releases the AK8963's
	 * data latch, as the datasheet requires. */
	if (imu_write(data, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_SLV_READ) != HAL_OK ||
	    imu_write(data, I2C_SLV0_REG, AK8963_ST1) != HAL_OK ||
	    imu_write(data, I2C_SLV0_CTRL, I2C_SLV_EN | MAG_EXT_LEN) != HAL_OK ||
	    imu_write(data, I2C_SLV4_CTRL, MAG_POLL_DLY) != HAL_OK ||
	    imu_write(data, I2C_MST_DELAY_CTRL,
	              I2C_MST_DELAY_ES_SHADOW | I2C_MST_SLV0_DLY_EN) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	HAL_Delay(20);   /* let the first poll land before anyone reads it */

	return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* Decoding                                                                  */
/* ------------------------------------------------------------------------- */

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

/* ext = EXT_SENS_DATA_00..07 = ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2. */
static void decode_mag(MPU9250_Data *data, const uint8_t *ext)
{
	data->flags &= (uint8_t)~(TLM_FLAG_MAG_STALE | TLM_FLAG_MAG_OVERFLOW);

	if (!mag_ready) {
		data->flags |= TLM_FLAG_MAG_STALE;
		return;
	}
	if (!(ext[0] & AK8963_ST1_DRDY)) {
		/* No new sample in this poll. The previous one stays in place and the
		 * flag tells the host it is not fresh. Note the converse is weaker:
		 * EXT_SENS_DATA changes at 100 Hz, so a faster loop reads the same
		 * fresh-flagged sample more than once. */
		data->flags |= TLM_FLAG_MAG_STALE;
		return;
	}
	if (ext[7] & AK8963_ST2_HOFL) {
		/* Overflow means the field exceeded the sensor's range, usually from
		 * something magnetic nearby. The reading is not merely noisy, it is
		 * meaningless, so discard it rather than filtering it. */
		data->flags |= TLM_FLAG_MAG_OVERFLOW | TLM_FLAG_MAG_STALE;
		return;
	}

	{
		float mx = (float)le16(&ext[1]) * MAG_LSB_UT * data->mag_adj[0];
		float my = (float)le16(&ext[3]) * MAG_LSB_UT * data->mag_adj[1];
		float mz = (float)le16(&ext[5]) * MAG_LSB_UT * data->mag_adj[2];

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
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

IMU_Status IMU_Init(MPU9250_Data *data)
{
	uint8_t who = 0;

	for (int i = 0; i < 3; i++) {
		data->mag_adj[i] = 1.0f;
		data->gyro_bias[i] = 0.0f;
	}
	data->bus_errors = 0;
	data->consecutive_errors = 0;
	data->flags = TLM_FLAG_UNCALIBRATED;
	data->calibrated = 0;
	data->mx = data->my = data->mz = 0.0f;
	mag_ready = 0;

	if (spi_init() != HAL_OK) {
		return IMU_ERR_BUS;
	}

	if (imu_read(data, WHO_AM_I_REG, &who, 1) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	if (who != MPU9250_WHOAMI_MPU9250 && who != MPU9250_WHOAMI_MPU9255 &&
	    who != MPU9250_WHOAMI_CLONE) {
		return IMU_ERR_WHOAMI;
	}

	/* Reset, then wait out the startup time before touching anything else. */
	if (imu_write(data, PWR_MGMT_1, 0x80) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	HAL_Delay(100);

	/* The reset re-enables the I2C slave interface. Turn it off at once: in
	 * SPI mode it can only cause trouble, by mistaking SPI traffic for I2C. */
	if (imu_write(data, USER_CTRL, USER_CTRL_I2C_IF_DIS) != HAL_OK) {
		return IMU_ERR_BUS;
	}

	/*
	 * Clock source 1 = PLL referenced to the X gyro. The reset default of 0
	 * uses the internal 20 MHz RC oscillator, whose frequency wanders with
	 * temperature -- so the sample interval wanders too, and the gyro
	 * integration inherits a scale error that looks exactly like drift.
	 */
	if (imu_write(data, PWR_MGMT_1, 0x01) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	HAL_Delay(10);
	if (imu_write(data, PWR_MGMT_2, 0x00) != HAL_OK) {
		return IMU_ERR_BUS;
	}

	/*
	 * DLPF_CFG 3 gives a 41 Hz gyro bandwidth at a 1 kHz internal rate, and
	 * SMPLRT_DIV 0 keeps the output there. 41 Hz sits above the attitude
	 * dynamics we care about and well below the propeller vibration that will
	 * dominate this signal once motors are turning. The 1 kHz rate also
	 * clocks the internal I2C master that polls the magnetometer.
	 */
	if (imu_write(data, MPU_CONFIG, 0x03) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	if (imu_write(data, SMPLRT_DIV, 0x00) != HAL_OK) {
		return IMU_ERR_BUS;
	}

	/*
	 * Write the ranges rather than reading back whatever happened to be in
	 * the registers. The low two bits of GYRO_CONFIG are FCHOICE_B, left at
	 * zero so the DLPF configured above is actually used -- setting them
	 * bypasses it and returns the raw 32 kHz signal.
	 */
	if (imu_write(data, GYRO_CONFIG, (uint8_t)(GYRO_FS_SEL << 3)) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	if (imu_write(data, ACCEL_CONFIG, (uint8_t)(ACCEL_FS_SEL << 3)) != HAL_OK) {
		return IMU_ERR_BUS;
	}
	/* Accelerometer DLPF, 41 Hz, matching the gyro. */
	if (imu_write(data, ACCEL_CONFIG2, 0x03) != HAL_OK) {
		return IMU_ERR_BUS;
	}

	data->accel_sens = ACCEL_SENS_LSB_PER_G[ACCEL_FS_SEL];
	data->gyro_sens = GYRO_SENS_LSB_PER_DPS[GYRO_FS_SEL];

	HAL_Delay(10);

	{
		IMU_Status mag_status = mag_init(data);
		mag_ready = (mag_status == IMU_OK);
		return mag_status;
	}
}

IMU_Status IMU_CalibrateGyro(MPU9250_Data *data)
{
	uint8_t raw[6];
	float sum[3] = {0.0f, 0.0f, 0.0f};
	uint32_t taken = 0;

	for (int i = 0; i < 3; i++) {
		data->gyro_bias[i] = 0.0f;
	}

	for (uint32_t i = 0; i < IMU_GYRO_CAL_SAMPLES; i++) {
		if (imu_read(data, GYRO_XOUT_H, raw, 6) == HAL_OK) {
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
		return IMU_ERR_BUS;
	}

	for (int i = 0; i < 3; i++) {
		data->gyro_bias[i] = (sum[i] / (float)taken) / data->gyro_sens;
	}
	data->calibrated = 1;
	data->flags &= (uint8_t)~TLM_FLAG_UNCALIBRATED;

	return IMU_OK;
}

IMU_Status IMU_ReadAll(MPU9250_Data *data)
{
	/* ACCEL_XOUT_H .. EXT_SENS_DATA_07 in one transaction: six accel bytes,
	 * two temperature, six gyro, eight magnetometer. One read is faster, and
	 * more importantly accel and gyro describe the same instant -- which is
	 * what the filter assumes when it fuses them. */
	uint8_t raw[IMU_BURST_LEN];

	if (imu_read(data, ACCEL_XOUT_H, raw, IMU_BURST_LEN) != HAL_OK) {
		data->flags |= TLM_FLAG_IMU_ERROR | TLM_FLAG_MAG_STALE;
		return IMU_ERR_BUS;
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

	decode_mag(data, &raw[14]);

	return IMU_OK;
}
