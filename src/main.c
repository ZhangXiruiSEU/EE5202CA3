/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "HARnet/network.h"
#include "sensor_drdy.h"

/* One scheduling reference window: 4 seconds expressed in microseconds. */
#define MAJOR_CYCLE_US 4000000ULL
/* Target accelerometer cadence: exactly 104 samples in each 4-second window. */
#define ACCEL_EVENTS_PER_MAJOR 104U
/*
 * Integer part of the accel interval:
 *   4,000,000 us / 104 = 38,461 us remainder 56.
 */
#define ACCEL_INTERVAL_BASE_US (MAJOR_CYCLE_US / ACCEL_EVENTS_PER_MAJOR)
/*
 * Fractional part of the accel interval, kept as a remainder instead of using
 * floating point. The scheduler accumulates this value and converts every 104
 * remainder units into one extra microsecond.
 */
#define ACCEL_INTERVAL_REM_US (MAJOR_CYCLE_US % ACCEL_EVENTS_PER_MAJOR)
#define GYRO_PERIOD_US 80000ULL
#define MAG_PERIOD_US 800000ULL
#define ENV_PERIOD_US 1000000ULL
#define HAR_PERIOD_US 1000000ULL
#define PRINT_PERIOD_US 1000000ULL
#define UART_BAUD_RATE 115200ULL
#define STATUS_MSG_SIZE 384
#define HTS221_PHASE_US 200000ULL
#define LPS22HB_PHASE_US 400000ULL
#define LIS3MDL_PHASE_US 600000ULL
#define HAR_PHASE_US 900000ULL
#define PRINT_PHASE_US 950000ULL
#define HAR_WINDOW_SAMPLES 26
#define HAR_AXES 3
#define HAR_INPUT_SCALE_MG 4000.0f

/*
 * Shared application state.
 *
 * Sensor tasks update the latest reading in this structure. The print task then
 * formats those cached values, so each sensor can run at its own rate without
 * forcing every output line to fetch every sensor again.
 */
struct app_context {
	const struct device *lsm6dsl_dev;
	const struct device *hts221_dev;
	const struct device *lis3mdl_dev;
	const struct device *lps22hb_dev;
	struct sensor_value accel[3];
	struct sensor_value humidity;
	struct sensor_value temperature;
	struct sensor_value pressure;
	struct sensor_value gyro[3];
	struct sensor_value magnetic[3];
	uint32_t har_cycles;
	uint64_t har_us;
	int har_status;
	float har_scores[3];
};

typedef void (*scheduler_task_fn)(struct app_context *ctx);

/*
 * Small cooperative scheduler entry.
 *
 * Each task owns a period and the absolute uptime when it should run next. The
 * main loop sleeps until the earliest release time, runs all due tasks, then
 * advances each task by whole periods so a delayed iteration does not execute a
 * burst of stale releases.
 */
struct scheduler_task {
	uint64_t period_us;
	uint64_t next_release_us;
	scheduler_task_fn run;
};

STAI_NETWORK_CONTEXT_DECLARE(har_network_ctx, STAI_NETWORK_CONTEXT_SIZE);
static STAI_ALIGNED(8) uint8_t har_activations[STAI_NETWORK_ACTIVATIONS_SIZE_BYTES];
static float har_window[HAR_WINDOW_SAMPLES][HAR_AXES];
static float *har_input;
static float *har_output;
static char status_msg[STATUS_MSG_SIZE];
static char drdy_msg[192];
static size_t har_count;
static size_t har_write_index;
/*
 * Device pointers are resolved from devicetree at build time. main() still
 * checks device_is_ready() before using them because probe or bus setup can
 * fail at runtime.
 */
static struct app_context app = {
	.lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl),
	.hts221_dev = DEVICE_DT_GET_ONE(st_hts221),
	.lis3mdl_dev = DEVICE_DT_GET_ONE(st_lis3mdl_magn),
	.lps22hb_dev = DEVICE_DT_GET_ONE(st_lps22hb_press),
};
static struct scheduler_task scheduler_tasks[6];

static uint64_t now_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

static const char *const har_labels[3] = {
	"running",
	"walking",
	"stationary",
};
static void append_fmt(char *buf, size_t buf_size, size_t *offset, const char *fmt, ...);

static void accel_schedule_advance(uint64_t *next_accel_us, uint32_t *accel_remainder_accum)
{
	/*
	 * Advance the next accelerometer release by the exact average interval
	 * without using floating point time.
	 *
	 * Desired interval:
	 *   4,000,000 us / 104 = 38,461.538461... us
	 *
	 * Integer scheduling cannot represent the .538461... us fraction directly,
	 * so each call adds 38,461 us and stores the dropped remainder, 56/104 us,
	 * in accel_remainder_accum. Whenever the accumulated remainder reaches 104,
	 * that represents one whole microsecond and is added back to next_accel_us.
	 *
	 * This makes individual intervals alternate between 38,461 us and 38,462 us,
	 * while every 104 intervals still add up to exactly 4,000,000 us.
	 */
	*next_accel_us += ACCEL_INTERVAL_BASE_US;
	*accel_remainder_accum += ACCEL_INTERVAL_REM_US;
	if (*accel_remainder_accum >= ACCEL_EVENTS_PER_MAJOR) {
		*next_accel_us += 1U;
		*accel_remainder_accum -= ACCEL_EVENTS_PER_MAJOR;
	}
}

static void append_fmt(char *buf, size_t buf_size, size_t *offset, const char *fmt, ...)
{
	va_list args;
	int written;

	if (*offset >= buf_size) {
		return;
	}

	va_start(args, fmt);
	written = vsnprintk(buf + *offset, buf_size - *offset, fmt, args);
	va_end(args);

	if (written < 0) {
		return;
	}

	*offset += (size_t)written;
}

static void append_text(char *buf, size_t buf_size, size_t *offset, const char *text)
{
	append_fmt(buf, buf_size, offset, "%s", text);
}

static void append_sensor_value(char *buf, size_t buf_size, size_t *offset,
			       const struct sensor_value *value)
{
	int64_t micro = (int64_t)value->val1 * 1000000LL + value->val2;

	if (*offset >= buf_size) {
		return;
	}

	if (micro < 0) {
		append_fmt(buf, buf_size, offset, "-%lld.%06lld",
			  -micro / 1000000LL, -micro % 1000000LL);
	} else {
		append_fmt(buf, buf_size, offset, "%lld.%06lld",
			  micro / 1000000LL, micro % 1000000LL);
	}
}

static void append_score(char *buf, size_t buf_size, size_t *offset, float value)
{
	int scaled = (int)(value * 1000.0f + 0.5f);

	if (*offset >= buf_size) {
		return;
	}

	if (scaled < 0) {
		scaled = 0;
	}

	append_fmt(buf, buf_size, offset, "%d.%03d", scaled / 1000, scaled % 1000);
}

static float accel_to_model_input(const struct sensor_value *accel)
{
	/* Convert Zephyr's m/s^2 value to mg, then scale to the model input range. */
	return (float)sensor_ms2_to_mg(accel) / HAR_INPUT_SCALE_MG;
}

static int read_accel(const struct device *dev, struct sensor_value accel[3])
{
	int ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);

	if (ret < 0) {
		printk("Cannot fetch accelerometer sample: %d\n", ret);
		return ret;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret < 0) {
		printk("Cannot read accelerometer channel: %d\n", ret);
		return ret;
	}

	return 0;
}

static void task_read_hts221(struct app_context *ctx)
{
	int ret;

	/*
	 * Poll the raw DRDY bit before fetching the sample. The fetch call may
	 * clear or consume data-ready state depending on the driver or sensor.
	 */
	sensor_drdy_poll_hts221();

	ret = sensor_sample_fetch(ctx->hts221_dev);
	if (ret < 0) {
		printk("HTS221 fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_HUMIDITY, &ctx->humidity);
	if (ret < 0) {
		printk("HTS221 humidity read failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_AMBIENT_TEMP, &ctx->temperature);
	if (ret < 0) {
		printk("HTS221 temperature read failed: %d\n", ret);
	}
}

static void task_read_gyro(struct app_context *ctx)
{
	int ret;

	/* Record whether the gyro had new data available at this scheduled slot. */
	sensor_drdy_poll_gyro();

	ret = sensor_sample_fetch_chan(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ);
	if (ret < 0) {
		printk("LSM6DSL gyro fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, ctx->gyro);
	if (ret < 0) {
		printk("LSM6DSL gyro read failed: %d\n", ret);
	}
}

static void task_read_lis3mdl(struct app_context *ctx)
{
	int ret;

	/* LIS3MDL exposes both data-ready and overrun bits; both are tracked. */
	sensor_drdy_poll_lis3mdl();

	ret = sensor_sample_fetch(ctx->lis3mdl_dev);
	if (ret < 0) {
		printk("LIS3MDL fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lis3mdl_dev, SENSOR_CHAN_MAGN_XYZ, ctx->magnetic);
	if (ret < 0) {
		printk("LIS3MDL read failed: %d\n", ret);
	}
}

static void task_read_lps22hb(struct app_context *ctx)
{
	int ret;

	/* LPS22HB pressure DRDY/overrun status is sampled before the driver fetch. */
	sensor_drdy_poll_lps22hb();

	ret = sensor_sample_fetch(ctx->lps22hb_dev);
	if (ret < 0) {
		printk("LPS22HB fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lps22hb_dev, SENSOR_CHAN_PRESS, &ctx->pressure);
	if (ret < 0) {
		printk("LPS22HB pressure read failed: %d\n", ret);
	}
}

static int har_init(void)
{
	stai_return_code rc;
	stai_ptr activations[] = { har_activations };
	stai_ptr inputs[STAI_NETWORK_IN_NUM];
	stai_ptr outputs[STAI_NETWORK_OUT_NUM];
	stai_size n_inputs;
	stai_size n_outputs;

	/*
	 * The generated HAR network owns static metadata, while this file provides
	 * the runtime context and activation memory required by ST AI.
	 */
	rc = stai_network_init(har_network_ctx);
	if (rc != STAI_SUCCESS) {
		printk("HAR init failed: 0x%x\n", rc);
		return -EIO;
	}

	rc = stai_network_set_activations(har_network_ctx, activations, ARRAY_SIZE(activations));
	if (rc != STAI_SUCCESS) {
		printk("HAR activations setup failed: 0x%x\n", rc);
		return -EIO;
	}

	rc = stai_network_get_inputs(har_network_ctx, inputs, &n_inputs);
	if (rc != STAI_SUCCESS || n_inputs != STAI_NETWORK_IN_NUM) {
		printk("HAR input lookup failed: 0x%x\n", rc);
		return -EIO;
	}

	rc = stai_network_get_outputs(har_network_ctx, outputs, &n_outputs);
	if (rc != STAI_SUCCESS || n_outputs != STAI_NETWORK_OUT_NUM) {
		printk("HAR output lookup failed: 0x%x\n", rc);
		return -EIO;
	}

	har_input = (float *)inputs[0];
	har_output = (float *)outputs[0];

	return 0;
}

static bool har_push_sample(const struct sensor_value accel[3])
{
	float sample[HAR_AXES] = {
		accel_to_model_input(&accel[0]),
		accel_to_model_input(&accel[1]),
		accel_to_model_input(&accel[2]),
	};

	/*
	 * Keep the latest HAR_WINDOW_SAMPLES acceleration samples in a ring buffer.
	 * har_write_index always points to the slot that will be overwritten next;
	 * once the buffer is full, that slot is also the oldest sample.
	 */
	memcpy(har_window[har_write_index], sample, sizeof(sample));
	har_write_index = (har_write_index + 1U) % HAR_WINDOW_SAMPLES;

	if (har_count < HAR_WINDOW_SAMPLES) {
		har_count++;
	}

	return har_count == HAR_WINDOW_SAMPLES;
}

static void task_accel_sample(struct app_context *ctx)
{
	/* Accel is the timing source for the HAR input window. */
	sensor_drdy_poll_accel();

	if (read_accel(ctx->lsm6dsl_dev, ctx->accel) == 0) {
		(void)har_push_sample(ctx->accel);
	}
}

static int har_run_inference_cached(struct app_context *ctx)
{
	stai_return_code rc;
	uint32_t start_cycles;
	uint32_t end_cycles;
	uint32_t elapsed_cycles;
	uint64_t elapsed_us;

	if (har_count < HAR_WINDOW_SAMPLES) {
		/* Not enough samples yet for one full model window. */
		ctx->har_status = 1;
		return 1;
	}

	/*
	 * Copy the ring buffer into the model input in chronological order:
	 * oldest sample first, newest sample last.
	 */
	for (size_t i = 0; i < HAR_WINDOW_SAMPLES; i++) {
		size_t src_index = (har_write_index + i) % HAR_WINDOW_SAMPLES;

		memcpy(&har_input[i * HAR_AXES], har_window[src_index], sizeof(har_window[0]));
	}

	/* Measure synchronous inference latency in both CPU cycles and microseconds. */
	start_cycles = k_cycle_get_32();
	rc = stai_network_run(har_network_ctx, STAI_MODE_SYNC);
	end_cycles = k_cycle_get_32();
	if (rc != STAI_SUCCESS) {
		printk("HAR inference failed: 0x%x\n", rc);
		ctx->har_status = -EIO;
		return -EIO;
	}

	elapsed_cycles = end_cycles - start_cycles;
	elapsed_us = k_cyc_to_us_floor64(elapsed_cycles);
	ctx->har_cycles = elapsed_cycles;
	ctx->har_us = elapsed_us;
	ctx->har_scores[0] = har_output[0];
	ctx->har_scores[1] = har_output[1];
	ctx->har_scores[2] = har_output[2];
	ctx->har_status = 0;
	return 0;
}

static void task_run_har_measure(struct app_context *ctx)
{
	(void)har_run_inference_cached(ctx);
}

static void task_print_status(struct app_context *ctx)
{
	size_t len = 0;

	/*
	 * Every 4 seconds the DRDY helper emits a compact health report and resets
	 * its counters. That report takes priority over the regular sensor line so
	 * the UART output remains bounded.
	 */
	sensor_drdy_format_report(drdy_msg, sizeof(drdy_msg), now_us());
	if (drdy_msg[0] != '\0') {
		printk("%s", drdy_msg);
		return;
	}

	append_fmt(status_msg, sizeof(status_msg), &len, "t=%llu ms | hum=",
		  (unsigned long long)k_uptime_get());
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->humidity);
	append_text(status_msg, sizeof(status_msg), &len, " % | temp=");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->temperature);
	append_text(status_msg, sizeof(status_msg), &len, " C | press=");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->pressure);
	append_text(status_msg, sizeof(status_msg), &len, " kPa | gyro=(");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->gyro[0]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->gyro[1]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->gyro[2]);
	append_text(status_msg, sizeof(status_msg), &len, ") rad/s | mag=(");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->magnetic[0]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->magnetic[1]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &ctx->magnetic[2]);
	append_text(status_msg, sizeof(status_msg), &len, ") gauss | har=");

	if (ctx->har_status == 0) {
		append_text(status_msg, sizeof(status_msg), &len, har_labels[0]);
		append_text(status_msg, sizeof(status_msg), &len, ":");
		append_score(status_msg, sizeof(status_msg), &len, ctx->har_scores[0]);
		append_text(status_msg, sizeof(status_msg), &len, ",");
		append_text(status_msg, sizeof(status_msg), &len, har_labels[1]);
		append_text(status_msg, sizeof(status_msg), &len, ":");
		append_score(status_msg, sizeof(status_msg), &len, ctx->har_scores[1]);
		append_text(status_msg, sizeof(status_msg), &len, ",");
		append_text(status_msg, sizeof(status_msg), &len, har_labels[2]);
		append_text(status_msg, sizeof(status_msg), &len, ":");
		append_score(status_msg, sizeof(status_msg), &len, ctx->har_scores[2]);
		append_fmt(status_msg, sizeof(status_msg), &len, " | infer=%llu.%03llu ms\n",
			  (unsigned long long)(ctx->har_us / 1000ULL),
			  (unsigned long long)(ctx->har_us % 1000ULL));
	} else {
		append_fmt(status_msg, sizeof(status_msg), &len, "waiting(%d/%d)\n",
			  (int)har_count, HAR_WINDOW_SAMPLES);
	}

	if (len >= sizeof(status_msg)) {
		len = sizeof(status_msg) - 1;
		status_msg[len] = '\0';
	}

	printk("%s", status_msg);
}

static void scheduler_init(struct scheduler_task *tasks, uint64_t start_us)
{
	/*
	 * Stagger low-rate tasks inside each one-second frame. This spreads I2C
	 * traffic, HAR inference, and UART formatting instead of doing them all at
	 * the same instant.
	 */
	tasks[0].period_us = GYRO_PERIOD_US;
	tasks[0].next_release_us = start_us;
	tasks[0].run = task_read_gyro;

	tasks[1].period_us = ENV_PERIOD_US;
	tasks[1].next_release_us = start_us + HTS221_PHASE_US;
	tasks[1].run = task_read_hts221;

	tasks[2].period_us = MAG_PERIOD_US;
	tasks[2].next_release_us = start_us + LIS3MDL_PHASE_US;
	tasks[2].run = task_read_lis3mdl;

	tasks[3].period_us = ENV_PERIOD_US;
	tasks[3].next_release_us = start_us + LPS22HB_PHASE_US;
	tasks[3].run = task_read_lps22hb;

	tasks[4].period_us = HAR_PERIOD_US;
	tasks[4].next_release_us = start_us + HAR_PHASE_US;
	tasks[4].run = task_run_har_measure;

	tasks[5].period_us = PRINT_PERIOD_US;
	tasks[5].next_release_us = start_us + PRINT_PHASE_US;
	tasks[5].run = task_print_status;
}

static uint64_t scheduler_next_wakeup(const struct scheduler_task *tasks, size_t task_count,
				      uint64_t next_accel_us)
{
	uint64_t next_wakeup_us = next_accel_us;

	for (size_t i = 0; i < task_count; i++) {
		if (tasks[i].next_release_us < next_wakeup_us) {
			next_wakeup_us = tasks[i].next_release_us;
		}
	}

	return next_wakeup_us;
}

static void scheduler_run_strict(struct app_context *ctx)
{
	uint64_t next_accel_us = now_us();
	uint32_t accel_remainder_accum = 0;

	scheduler_init(scheduler_tasks, next_accel_us);

	/*
	 * Single-thread cooperative scheduler:
	 * - keep an absolute "next release" timestamp for each task;
	 * - sleep until the earliest pending release;
	 * - run every task that is due;
	 * - advance missed releases to the next future slot instead of backfilling.
	 *
	 * This keeps task phases stable over long runs. Accel sampling is scheduled
	 * separately because its exact period is fractional in microseconds.
	 */
	while (1) {
		uint64_t current_us = now_us();
		uint64_t next_wakeup_us = scheduler_next_wakeup(scheduler_tasks,
						 ARRAY_SIZE(scheduler_tasks),
						 next_accel_us);

		/* Sleep until the next scheduled release instead of busy waiting. */
		if (current_us < next_wakeup_us) {
			uint64_t sleep_us = next_wakeup_us - current_us;

			k_sleep(K_USEC((int32_t)sleep_us));
			continue;
		}

		if (current_us >= next_accel_us) {
			task_accel_sample(ctx);
			do {
				accel_schedule_advance(&next_accel_us, &accel_remainder_accum);
			} while (current_us >= next_accel_us);
		}

		for (size_t i = 0; i < ARRAY_SIZE(scheduler_tasks); i++) {
			current_us = now_us();
			if (current_us >= scheduler_tasks[i].next_release_us) {
				scheduler_tasks[i].run(ctx);
				/*
				 * Skip stale releases and keep the task aligned to its
				 * original time grid rather than current_us + period.
				 */
				do {
					scheduler_tasks[i].next_release_us += scheduler_tasks[i].period_us;
				} while (current_us >= scheduler_tasks[i].next_release_us);
			}
		}
	}
}

int main(void)
{
	struct sensor_value accel_odr_attr = { .val1 = 26, .val2 = 0 };
	struct sensor_value gyro_odr_attr = { .val1 = 12, .val2 = 0 };

	/* The LSM6DSL provides both accelerometer and gyro channels. */
	if (!device_is_ready(app.lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	/* Auxiliary sensors are required for the full status output. */
	if (!device_is_ready(app.hts221_dev) || !device_is_ready(app.lis3mdl_dev) ||
	    !device_is_ready(app.lps22hb_dev)) {
		printk("aux sensor: device not ready.\n");
		return 0;
	}

	/*
	 * Configure the physical LSM6DSL output data rates. The software scheduler
	 * then reads at matching or lower rates and uses DRDY counters to detect
	 * missed or duplicate samples.
	 */
	if (sensor_attr_set(app.lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &accel_odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	if (sensor_attr_set(app.lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &gyro_odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

	if (har_init() < 0) {
		return 0;
	}

	if (sensor_drdy_init() < 0) {
		printk("DRDY helper init failed.\n");
		return 0;
	}

	printk("HAR window: %d x %d accel samples\n", HAR_WINDOW_SAMPLES, HAR_AXES);
	/* Does not return during normal operation. */
	scheduler_run_strict(&app);
	return 0;
}
