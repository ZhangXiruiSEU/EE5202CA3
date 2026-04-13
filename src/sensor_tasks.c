#include "sensor_tasks.h"

#include <zephyr/sys/printk.h>
#include <string.h>

#include "lsm6dsl_fifo.h"
#include "sensor_drdy.h"

static int read_current_accel(const struct device *dev, struct sensor_value accel[3])
{
	int ret;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	if (ret < 0) {
		k_mutex_unlock(&sensor_lock);
		printk("Cannot fetch accelerometer sample: %d\n", ret);
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	k_mutex_unlock(&sensor_lock);
	if (ret < 0) {
		printk("Cannot read accelerometer channel: %d\n", ret);
		return ret;
	}

	return 0;
}

static void enqueue_accel_sample(struct app_context *ctx, const struct accel_sample *sample)
{
	struct accel_sample dropped;

	if (!sensor_drdy_reserve_accel_in(sample->timestamp_us)) {
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	memcpy(ctx->accel, sample->accel, sizeof(ctx->accel));
	if (sample->late_us > 0U) {
		ctx->accel_late_samples++;
		if (sample->late_us > ctx->accel_max_late_us) {
			ctx->accel_max_late_us = sample->late_us;
		}
	}
	k_mutex_unlock(&app_lock);

	if (k_msgq_put(&accel_msgq, sample, K_NO_WAIT) == 0) {
		return;
	}

	/* Cyclic buffering policy: make room by discarding the oldest sample. */
	if (k_msgq_get(&accel_msgq, &dropped, K_NO_WAIT) == 0 &&
	    k_msgq_put(&accel_msgq, sample, K_NO_WAIT) == 0) {
		k_mutex_lock(&app_lock, K_FOREVER);
		ctx->accel_queue_drops++;
		k_mutex_unlock(&app_lock);
	}
}

void task_accel_sample(struct app_context *ctx, uint64_t timestamp_us, uint32_t late_us)
{
	uint16_t fifo_samples;

	sensor_drdy_poll_accel();

	if (lsm6dsl_fifo_sample_count(&fifo_samples) == 0) {
		if (fifo_samples == 0U) {
			return;
		}

		for (uint16_t i = 0U; i < fifo_samples; i++) {
			uint64_t sample_age_us =
				(uint64_t)(fifo_samples - 1U - i) * ACCEL_INTERVAL_BASE_US;
			struct accel_sample sample = {
				.timestamp_us = (timestamp_us > sample_age_us) ?
						timestamp_us - sample_age_us : 0U,
				.late_us = (i == 0U) ? late_us : 0U,
			};

			if (lsm6dsl_fifo_read_accel(sample.accel) < 0) {
				return;
			}

			enqueue_accel_sample(ctx, &sample);
		}

		return;
	}

	/*
	 * If FIFO status cannot be read, keep the accel path alive by reading
	 * the current output registers once.
	 */
	struct accel_sample sample = {
		.timestamp_us = timestamp_us,
		.late_us = late_us,
	};

	if (read_current_accel(ctx->lsm6dsl_dev, sample.accel) != 0) {
		return;
	}

	enqueue_accel_sample(ctx, &sample);
}

void task_read_hts221(struct app_context *ctx)
{
	int ret;
	struct sensor_value humidity;
	struct sensor_value temperature;

	sensor_drdy_poll_hts221();

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = sensor_sample_fetch(ctx->hts221_dev);
	if (ret < 0) {
		k_mutex_unlock(&sensor_lock);
		printk("HTS221 fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	if (ret < 0) {
		k_mutex_unlock(&sensor_lock);
		printk("HTS221 humidity read failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	k_mutex_unlock(&sensor_lock);
	if (ret < 0) {
		printk("HTS221 temperature read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->humidity = humidity;
	ctx->temperature = temperature;
	k_mutex_unlock(&app_lock);
}

void task_read_gyro(struct app_context *ctx)
{
	int ret;
	struct sensor_value gyro[3];

	sensor_drdy_poll_gyro();

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = sensor_sample_fetch_chan(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ);
	if (ret < 0) {
		k_mutex_unlock(&sensor_lock);
		printk("LSM6DSL gyro fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	k_mutex_unlock(&sensor_lock);
	if (ret < 0) {
		printk("LSM6DSL gyro read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	memcpy(ctx->gyro, gyro, sizeof(ctx->gyro));
	k_mutex_unlock(&app_lock);
}

void task_read_lis3mdl(struct app_context *ctx)
{
	int ret;
	struct sensor_value magnetic[3];

	sensor_drdy_poll_lis3mdl();

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = sensor_sample_fetch(ctx->lis3mdl_dev);
	if (ret < 0) {
		k_mutex_unlock(&sensor_lock);
		printk("LIS3MDL fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lis3mdl_dev, SENSOR_CHAN_MAGN_XYZ, magnetic);
	k_mutex_unlock(&sensor_lock);
	if (ret < 0) {
		printk("LIS3MDL read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	memcpy(ctx->magnetic, magnetic, sizeof(ctx->magnetic));
	k_mutex_unlock(&app_lock);
}

void task_read_lps22hb(struct app_context *ctx)
{
	int ret;
	struct sensor_value pressure;

	sensor_drdy_poll_lps22hb();

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = sensor_sample_fetch(ctx->lps22hb_dev);
	if (ret < 0) {
		k_mutex_unlock(&sensor_lock);
		printk("LPS22HB fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lps22hb_dev, SENSOR_CHAN_PRESS, &pressure);
	k_mutex_unlock(&sensor_lock);
	if (ret < 0) {
		printk("LPS22HB pressure read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->pressure = pressure;
	k_mutex_unlock(&app_lock);
}
