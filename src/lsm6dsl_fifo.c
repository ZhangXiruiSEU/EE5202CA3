#include "lsm6dsl_fifo.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "app_context.h"

/* FIFO threshold low byte. We clear it because this app polls instead of using a watermark. */
#define LSM6DSL_REG_FIFO_CTRL1 0x06

/* FIFO threshold high bits and related FIFO control bits. Also cleared for no watermark. */
#define LSM6DSL_REG_FIFO_CTRL2 0x07

/* FIFO decimation selector: chooses accel/gyro streams and their decimation. */
#define LSM6DSL_REG_FIFO_CTRL3 0x08

/* FIFO ODR and FIFO operating mode, such as bypass or stream mode. */
#define LSM6DSL_REG_FIFO_CTRL5 0x0A

/* FIFO unread level low byte; burst-read with STATUS2 to get the full level. */
#define LSM6DSL_REG_FIFO_STATUS1 0x3A

/* FIFO data output start register; burst reads consume samples from hardware FIFO. */
#define LSM6DSL_REG_FIFO_DATA_OUT_L 0x3E

/* Put every accel sample into FIFO; no accelerometer decimation. */
#define LSM6DSL_FIFO_XL_NO_DEC 1U

/* Do not store gyro samples in FIFO because HAR only needs acceleration. */
#define LSM6DSL_FIFO_GY_DISABLE 0U

/* FIFO output data rate code for 26 Hz. */
#define LSM6DSL_FIFO_26HZ 2U

/* FIFO mode code that keeps accepting new samples after it becomes non-empty. */
#define LSM6DSL_FIFO_STREAM_MODE 6U

/* FIFO mode code used to stop/reset FIFO before reprogramming it. */
#define LSM6DSL_FIFO_BYPASS_MODE 0U

/* One accel sample is X/Y/Z, each stored as one 16-bit FIFO word. */
#define LSM6DSL_FIFO_WORDS_PER_ACCEL_SAMPLE 3U

/* Default +-2g accel sensitivity from the LSM6DSL driver: 61 micro-g per LSB. */
#define LSM6DSL_ACCEL_SENSITIVITY_UG_PER_LSB 61LL

static const struct i2c_dt_spec lsm6dsl_i2c = I2C_DT_SPEC_GET(DT_INST(0, st_lsm6dsl));

/*
 * Write one LSM6DSL register through I2C.
 *
 * This file talks to FIFO registers directly because Zephyr's generic sensor
 * API does not expose a portable "drain all FIFO samples" call. The shared
 * sensor_lock keeps these direct I2C transfers from racing Zephyr driver calls
 * that access the same LSM6DSL device.
 */
static int fifo_reg_write(uint8_t reg, uint8_t value)
{
	int ret;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = i2c_reg_write_byte_dt(&lsm6dsl_i2c, reg, value);
	k_mutex_unlock(&sensor_lock);

	return ret;
}

/*
 * Read a consecutive register block from the LSM6DSL.
 *
 * FIFO status and FIFO data are both read as bursts. Reading FIFO_DATA_OUT_L
 * advances the sensor's hardware FIFO read pointer, so callers must only call
 * this when they intend to consume a sample.
 */
static int fifo_burst_read(uint8_t reg, uint8_t *buf, size_t len)
{
	int ret;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = i2c_burst_read_dt(&lsm6dsl_i2c, reg, buf, len);
	k_mutex_unlock(&sensor_lock);

	return ret;
}

/*
 * Convert one raw LSM6DSL accel axis from FIFO format into Zephyr's
 * sensor_value unit format. FIFO accel samples use the same raw scale as the
 * normal accel output registers.
 */
static void raw_accel_to_sensor_value(int16_t raw, struct sensor_value *value)
{
	/* Default LSM6DSL accel full-scale is 2g: 61 ug/LSB in Zephyr's driver. */
	sensor_ug_to_ms2((int64_t)raw * LSM6DSL_ACCEL_SENSITIVITY_UG_PER_LSB, value);
}

/*
 * Configure the LSM6DSL hardware FIFO for this application.
 *
 * The FIFO is accel-only because HAR only consumes acceleration. It runs at
 * the same 26 Hz rate as the accel sampling thread, so if the urgent thread
 * blocks the CPU for one or more periods, the sensor can still collect samples
 * internally until the accel thread gets CPU time again.
 */
int lsm6dsl_fifo_init(void)
{
	if (!i2c_is_ready_dt(&lsm6dsl_i2c)) {
		return -ENODEV;
	}

	/*
	 * Program FIFO for accel-only stream mode. The FIFO keeps sampling while
	 * the CPU is blocked, and the accel thread drains pending samples later.
	 */
	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL5, LSM6DSL_FIFO_BYPASS_MODE) < 0) {
		return -EIO;
	}

	/* No explicit FIFO threshold; the accel thread polls and drains by count. */
	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL1, 0U) < 0 ||
	    fifo_reg_write(LSM6DSL_REG_FIFO_CTRL2, 0U) < 0) {
		return -EIO;
	}

	/* Store accel data in FIFO without decimation; do not store gyro data. */
	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL3,
			   (LSM6DSL_FIFO_GY_DISABLE << 3) | LSM6DSL_FIFO_XL_NO_DEC) < 0) {
		return -EIO;
	}

	/* Start FIFO stream mode at 26 Hz. */
	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL5,
			   (LSM6DSL_FIFO_26HZ << 3) | LSM6DSL_FIFO_STREAM_MODE) < 0) {
		return -EIO;
	}

	return 0;
}

/*
 * Return how many complete accel samples are currently waiting in hardware FIFO.
 *
 * LSM6DSL reports FIFO level in 16-bit words. One accel sample is X/Y/Z, so it
 * consumes three FIFO words. Any incomplete tail is ignored until the next poll.
 */
int lsm6dsl_fifo_sample_count(uint16_t *sample_count)
{
	uint8_t status[2];
	uint16_t fifo_words;

	if (fifo_burst_read(LSM6DSL_REG_FIFO_STATUS1, status, sizeof(status)) < 0) {
		return -EIO;
	}

	fifo_words = (uint16_t)status[0] | ((uint16_t)(status[1] & 0x07U) << 8);
	*sample_count = fifo_words / LSM6DSL_FIFO_WORDS_PER_ACCEL_SAMPLE;

	return 0;
}

/*
 * Consume one accel sample from the LSM6DSL hardware FIFO.
 *
 * Each sample is 6 bytes: little-endian signed 16-bit X, Y, and Z. Calling this
 * repeatedly drains older samples first.
 */
int lsm6dsl_fifo_read_accel(struct sensor_value accel[3])
{
	uint8_t raw[6];

	if (fifo_burst_read(LSM6DSL_REG_FIFO_DATA_OUT_L, raw, sizeof(raw)) < 0) {
		return -EIO;
	}

	raw_accel_to_sensor_value((int16_t)sys_get_le16(&raw[0]), &accel[0]);
	raw_accel_to_sensor_value((int16_t)sys_get_le16(&raw[2]), &accel[1]);
	raw_accel_to_sensor_value((int16_t)sys_get_le16(&raw[4]), &accel[2]);

	return 0;
}
