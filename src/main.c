/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>

#include "app_context.h"
#include "app_threads.h"
#include "har_task.h"
#include "lsm6dsl_fifo.h"
#include "sensor_drdy.h"

static struct app_context app = {
	.lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl),
	.hts221_dev = DEVICE_DT_GET_ONE(st_hts221),
	.lis3mdl_dev = DEVICE_DT_GET_ONE(st_lis3mdl_magn),
	.lps22hb_dev = DEVICE_DT_GET_ONE(st_lps22hb_press),
	.har_status = 1,
};

/* These macros allocate and statically initialize the shared kernel objects. */
K_MUTEX_DEFINE(app_lock);
K_MUTEX_DEFINE(sensor_lock);
/* Fixed-depth producer/consumer queue from the accel thread to the HAR thread. */
K_MSGQ_DEFINE(accel_msgq, sizeof(struct accel_sample), ACCEL_QUEUE_DEPTH, 4);

static int init_devices(struct app_context *ctx)
{
	struct sensor_value accel_odr_attr = { .val1 = 26, .val2 = 0 };
	struct sensor_value gyro_odr_attr = { .val1 = 12, .val2 = 0 };

	if (!device_is_ready(ctx->lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
		return -ENODEV;
	}

	if (!device_is_ready(ctx->hts221_dev) || !device_is_ready(ctx->lis3mdl_dev) ||
	    !device_is_ready(ctx->lps22hb_dev)) {
		printk("aux sensor: device not ready.\n");
		return -ENODEV;
	}

	/*
	 * Keep the HAR input rate explicit in code: the model consumes a
	 * 26-sample accelerometer window once per second.
	 */
	if (sensor_attr_set(ctx->lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &accel_odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return -EIO;
	}

	/*
	 * The gyroscope shares the LSM6DSL device but has its own channel ODR.
	 * Lower-rate auxiliary sensors keep their default ODRs from prj.conf.
	 */
	if (sensor_attr_set(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &gyro_odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return -EIO;
	}

	return 0;
}

int main(void)
{
	if (init_devices(&app) < 0) {
		return 0;
	}

	if (har_init() < 0) {
		return 0;
	}

	if (sensor_drdy_init() < 0) {
		printk("DRDY helper init failed.\n");
		return 0;
	}

	if (lsm6dsl_fifo_init() < 0) {
		printk("LSM6DSL FIFO init failed.\n");
		return 0;
	}

	printk("HAR window: %d x %d accel samples\n", HAR_WINDOW_SAMPLES, HAR_AXES);
	app_threads_start(&app);

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
