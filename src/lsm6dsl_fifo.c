#include "lsm6dsl_fifo.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "app_context.h"

#define LSM6DSL_REG_FIFO_CTRL1 0x06
#define LSM6DSL_REG_FIFO_CTRL2 0x07
#define LSM6DSL_REG_FIFO_CTRL3 0x08
#define LSM6DSL_REG_FIFO_CTRL5 0x0A
#define LSM6DSL_REG_FIFO_STATUS1 0x3A
#define LSM6DSL_REG_FIFO_DATA_OUT_L 0x3E

#define LSM6DSL_FIFO_XL_NO_DEC 1U
#define LSM6DSL_FIFO_GY_DISABLE 0U
#define LSM6DSL_FIFO_26HZ 2U
#define LSM6DSL_FIFO_STREAM_MODE 6U
#define LSM6DSL_FIFO_BYPASS_MODE 0U
#define LSM6DSL_FIFO_WORDS_PER_ACCEL_SAMPLE 3U
#define LSM6DSL_ACCEL_SENSITIVITY_UG_PER_LSB 61LL

static const struct i2c_dt_spec lsm6dsl_i2c = I2C_DT_SPEC_GET(DT_INST(0, st_lsm6dsl));

static int fifo_reg_write(uint8_t reg, uint8_t value)
{
	int ret;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = i2c_reg_write_byte_dt(&lsm6dsl_i2c, reg, value);
	k_mutex_unlock(&sensor_lock);

	return ret;
}

static int fifo_burst_read(uint8_t reg, uint8_t *buf, size_t len)
{
	int ret;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = i2c_burst_read_dt(&lsm6dsl_i2c, reg, buf, len);
	k_mutex_unlock(&sensor_lock);

	return ret;
}

static void raw_accel_to_sensor_value(int16_t raw, struct sensor_value *value)
{
	/* Default LSM6DSL accel full-scale is 2g: 61 ug/LSB in Zephyr's driver. */
	sensor_ug_to_ms2((int64_t)raw * LSM6DSL_ACCEL_SENSITIVITY_UG_PER_LSB, value);
}

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

	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL1, 0U) < 0 ||
	    fifo_reg_write(LSM6DSL_REG_FIFO_CTRL2, 0U) < 0) {
		return -EIO;
	}

	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL3,
			   (LSM6DSL_FIFO_GY_DISABLE << 3) | LSM6DSL_FIFO_XL_NO_DEC) < 0) {
		return -EIO;
	}

	if (fifo_reg_write(LSM6DSL_REG_FIFO_CTRL5,
			   (LSM6DSL_FIFO_26HZ << 3) | LSM6DSL_FIFO_STREAM_MODE) < 0) {
		return -EIO;
	}

	return 0;
}

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
