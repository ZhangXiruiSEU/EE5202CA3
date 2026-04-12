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

/* Spread 104 accel polls evenly across a 4 s major cycle. */
#define MAJOR_CYCLE_US 4000000ULL
#define ACCEL_EVENTS_PER_MAJOR 104U
#define ACCEL_INTERVAL_BASE_US (MAJOR_CYCLE_US / ACCEL_EVENTS_PER_MAJOR)
#define ACCEL_INTERVAL_REM_US (MAJOR_CYCLE_US % ACCEL_EVENTS_PER_MAJOR)
#define GYRO_PERIOD_US 80000ULL
#define MAG_PERIOD_US 800000ULL
#define ENV_PERIOD_US 1000000ULL
#define HAR_PERIOD_US 1000000ULL
#define PRINT_PERIOD_US 1000000ULL
#define STATUS_MSG_SIZE 384
#define HTS221_PHASE_US 200000ULL
#define LPS22HB_PHASE_US 400000ULL
#define LIS3MDL_PHASE_US 600000ULL
#define HAR_PHASE_US 900000ULL
#define PRINT_PHASE_US 950000ULL
#define HAR_WINDOW_SAMPLES 26
#define HAR_AXES 3
#define HAR_INPUT_SCALE_MG 4000.0f
#define ACCEL_QUEUE_DEPTH 128
#define URGENT_BLOCK_MIN_US 30000U
#define URGENT_BLOCK_MAX_US 50000U
#define URGENT_SLEEP_MIN_MS 250U
#define URGENT_SLEEP_MAX_MS 900U

#define URGENT_THREAD_PRIO 0
#define ACCEL_THREAD_PRIO 1
#define GYRO_THREAD_PRIO 2
#define AUX_THREAD_PRIO 3
#define HAR_THREAD_PRIO 4
#define PRINT_THREAD_PRIO 5

#define URGENT_STACK_SIZE 1024
#define SENSOR_STACK_SIZE 2048
#define HAR_STACK_SIZE 4096
#define PRINT_STACK_SIZE 2048

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
	size_t har_sample_count;
	uint32_t accel_queue_drops;
	uint32_t accel_late_samples;
	uint32_t accel_max_late_us;
	uint32_t urgent_events;
	uint32_t urgent_last_block_us;
};

struct accel_sample {
	struct sensor_value accel[3];
	uint64_t timestamp_us;
	uint32_t late_us;
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
static struct app_context app = {
	.lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl),
	.hts221_dev = DEVICE_DT_GET_ONE(st_hts221),
	.lis3mdl_dev = DEVICE_DT_GET_ONE(st_lis3mdl_magn),
	.lps22hb_dev = DEVICE_DT_GET_ONE(st_lps22hb_press),
	.har_status = 1,
};

K_MUTEX_DEFINE(app_lock);
K_MUTEX_DEFINE(sensor_lock);
K_MSGQ_DEFINE(accel_msgq, sizeof(struct accel_sample), ACCEL_QUEUE_DEPTH, 4);

K_THREAD_STACK_DEFINE(urgent_stack, URGENT_STACK_SIZE);
K_THREAD_STACK_DEFINE(accel_stack, SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(gyro_stack, SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(hts221_stack, SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(lis3mdl_stack, SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(lps22hb_stack, SENSOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(har_stack, HAR_STACK_SIZE);
K_THREAD_STACK_DEFINE(print_stack, PRINT_STACK_SIZE);

static struct k_thread urgent_thread_data;
static struct k_thread accel_thread_data;
static struct k_thread gyro_thread_data;
static struct k_thread hts221_thread_data;
static struct k_thread lis3mdl_thread_data;
static struct k_thread lps22hb_thread_data;
static struct k_thread har_thread_data;
static struct k_thread print_thread_data;

static uint64_t now_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

static uint32_t pseudo_random32(void)
{
	static uint32_t state;

	if (state == 0U) {
		state = k_cycle_get_32() ^ 0xa5a55a5aU;
	}

	state = state * 1664525U + 1013904223U;
	return state;
}

static const char *const har_labels[3] = {
	"running",
	"walking",
	"stationary",
};
static void append_fmt(char *buf, size_t buf_size, size_t *offset, const char *fmt, ...);

static void accel_schedule_advance(uint64_t *next_accel_us, uint32_t *accel_remainder_accum)
{
	/* Use integer remainder accumulation to avoid long-term drift. */
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
	/* Zephyr sensor values store the fractional part in micro-units. */
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

static void sleep_until_us(uint64_t target_us)
{
	uint64_t current_us = now_us();

	if (current_us < target_us) {
		k_sleep(K_USEC((int32_t)(target_us - current_us)));
	}
}

static float accel_to_model_input(const struct sensor_value *accel)
{
	/* Match the normalization used when the HAR model was trained. */
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
	struct sensor_value humidity;
	struct sensor_value temperature;

	sensor_drdy_poll_hts221();

	ret = sensor_sample_fetch(ctx->hts221_dev);
	if (ret < 0) {
		printk("HTS221 fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	if (ret < 0) {
		printk("HTS221 humidity read failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	if (ret < 0) {
		printk("HTS221 temperature read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->humidity = humidity;
	ctx->temperature = temperature;
	k_mutex_unlock(&app_lock);
}

static void task_read_gyro(struct app_context *ctx)
{
	int ret;
	struct sensor_value gyro[3];

	sensor_drdy_poll_gyro();

	ret = sensor_sample_fetch_chan(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ);
	if (ret < 0) {
		printk("LSM6DSL gyro fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret < 0) {
		printk("LSM6DSL gyro read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	memcpy(ctx->gyro, gyro, sizeof(ctx->gyro));
	k_mutex_unlock(&app_lock);
}

static void task_read_lis3mdl(struct app_context *ctx)
{
	int ret;
	struct sensor_value magnetic[3];

	sensor_drdy_poll_lis3mdl();

	ret = sensor_sample_fetch(ctx->lis3mdl_dev);
	if (ret < 0) {
		printk("LIS3MDL fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lis3mdl_dev, SENSOR_CHAN_MAGN_XYZ, magnetic);
	if (ret < 0) {
		printk("LIS3MDL read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	memcpy(ctx->magnetic, magnetic, sizeof(ctx->magnetic));
	k_mutex_unlock(&app_lock);
}

static void task_read_lps22hb(struct app_context *ctx)
{
	int ret;
	struct sensor_value pressure;

	sensor_drdy_poll_lps22hb();

	ret = sensor_sample_fetch(ctx->lps22hb_dev);
	if (ret < 0) {
		printk("LPS22HB fetch failed: %d\n", ret);
		return;
	}

	ret = sensor_channel_get(ctx->lps22hb_dev, SENSOR_CHAN_PRESS, &pressure);
	if (ret < 0) {
		printk("LPS22HB pressure read failed: %d\n", ret);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->pressure = pressure;
	k_mutex_unlock(&app_lock);
}

static int har_init(void)
{
	stai_return_code rc;
	stai_ptr activations[] = { har_activations };
	stai_ptr inputs[STAI_NETWORK_IN_NUM];
	stai_ptr outputs[STAI_NETWORK_OUT_NUM];
	stai_size n_inputs;
	stai_size n_outputs;

	/* Resolve the generated model buffers once and reuse them forever. */
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

	memcpy(har_window[har_write_index], sample, sizeof(sample));
	har_write_index = (har_write_index + 1U) % HAR_WINDOW_SAMPLES;

	if (har_count < HAR_WINDOW_SAMPLES) {
		har_count++;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	app.har_sample_count = har_count;
	k_mutex_unlock(&app_lock);

	/* Inference can only start once the rolling window is completely filled. */
	return har_count == HAR_WINDOW_SAMPLES;
}

static void task_accel_sample(struct app_context *ctx, uint64_t timestamp_us, uint32_t late_us)
{
	struct accel_sample sample = {
		.timestamp_us = timestamp_us,
		.late_us = late_us,
	};
	struct accel_sample dropped;

	sensor_drdy_poll_accel();

	if (read_accel(ctx->lsm6dsl_dev, sample.accel) != 0) {
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	memcpy(ctx->accel, sample.accel, sizeof(ctx->accel));
	if (late_us > 0U) {
		ctx->accel_late_samples++;
		if (late_us > ctx->accel_max_late_us) {
			ctx->accel_max_late_us = late_us;
		}
	}
	k_mutex_unlock(&app_lock);

	if (k_msgq_put(&accel_msgq, &sample, K_NO_WAIT) == 0) {
		return;
	}

	/* Cyclic buffering policy: make room by discarding the oldest sample. */
	if (k_msgq_get(&accel_msgq, &dropped, K_NO_WAIT) == 0 &&
	    k_msgq_put(&accel_msgq, &sample, K_NO_WAIT) == 0) {
		k_mutex_lock(&app_lock, K_FOREVER);
		ctx->accel_queue_drops++;
		k_mutex_unlock(&app_lock);
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
		k_mutex_lock(&app_lock, K_FOREVER);
		ctx->har_status = 1;
		k_mutex_unlock(&app_lock);
		return 1;
	}

	/*
	 * The ring buffer wraps continuously, so rebuild a linear input tensor
	 * in oldest-to-newest order before invoking the network.
	 */
	for (size_t i = 0; i < HAR_WINDOW_SAMPLES; i++) {
		size_t src_index = (har_write_index + i) % HAR_WINDOW_SAMPLES;

		memcpy(&har_input[i * HAR_AXES], har_window[src_index], sizeof(har_window[0]));
	}

	start_cycles = k_cycle_get_32();
	rc = stai_network_run(har_network_ctx, STAI_MODE_SYNC);
	end_cycles = k_cycle_get_32();
	if (rc != STAI_SUCCESS) {
		printk("HAR inference failed: 0x%x\n", rc);
		k_mutex_lock(&app_lock, K_FOREVER);
		ctx->har_status = -EIO;
		k_mutex_unlock(&app_lock);
		return -EIO;
	}

	elapsed_cycles = end_cycles - start_cycles;
	elapsed_us = k_cyc_to_us_floor64(elapsed_cycles);
	k_mutex_lock(&app_lock, K_FOREVER);
	ctx->har_cycles = elapsed_cycles;
	ctx->har_us = elapsed_us;
	ctx->har_scores[0] = har_output[0];
	ctx->har_scores[1] = har_output[1];
	ctx->har_scores[2] = har_output[2];
	ctx->har_status = 0;
	k_mutex_unlock(&app_lock);
	return 0;
}

static void task_run_har_measure(struct app_context *ctx)
{
	struct accel_sample sample;

	while (k_msgq_get(&accel_msgq, &sample, K_NO_WAIT) == 0) {
		(void)har_push_sample(sample.accel);
	}

	(void)har_run_inference_cached(ctx);
}

static void task_print_status(struct app_context *ctx)
{
	size_t len = 0;
	struct app_context snapshot;

	sensor_drdy_format_report(drdy_msg, sizeof(drdy_msg), now_us());
	if (drdy_msg[0] != '\0') {
		printk("%s", drdy_msg);
		return;
	}

	k_mutex_lock(&app_lock, K_FOREVER);
	snapshot = *ctx;
	k_mutex_unlock(&app_lock);

	append_fmt(status_msg, sizeof(status_msg), &len, "t=%llu ms | hum=",
		  (unsigned long long)k_uptime_get());
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.humidity);
	append_text(status_msg, sizeof(status_msg), &len, " % | temp=");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.temperature);
	append_text(status_msg, sizeof(status_msg), &len, " C | press=");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.pressure);
	append_text(status_msg, sizeof(status_msg), &len, " kPa | gyro=(");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.gyro[0]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.gyro[1]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.gyro[2]);
	append_text(status_msg, sizeof(status_msg), &len, ") rad/s | mag=(");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.magnetic[0]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.magnetic[1]);
	append_text(status_msg, sizeof(status_msg), &len, ",");
	append_sensor_value(status_msg, sizeof(status_msg), &len, &snapshot.magnetic[2]);
	append_text(status_msg, sizeof(status_msg), &len, ") gauss | har=");

	if (snapshot.har_status == 0) {
		append_text(status_msg, sizeof(status_msg), &len, har_labels[0]);
		append_text(status_msg, sizeof(status_msg), &len, ":");
		append_score(status_msg, sizeof(status_msg), &len, snapshot.har_scores[0]);
		append_text(status_msg, sizeof(status_msg), &len, ",");
		append_text(status_msg, sizeof(status_msg), &len, har_labels[1]);
		append_text(status_msg, sizeof(status_msg), &len, ":");
		append_score(status_msg, sizeof(status_msg), &len, snapshot.har_scores[1]);
		append_text(status_msg, sizeof(status_msg), &len, ",");
		append_text(status_msg, sizeof(status_msg), &len, har_labels[2]);
		append_text(status_msg, sizeof(status_msg), &len, ":");
		append_score(status_msg, sizeof(status_msg), &len, snapshot.har_scores[2]);
		append_fmt(status_msg, sizeof(status_msg), &len,
			  " | infer=%llu.%03llu ms | qdrop=%u late=%u maxlate=%u us urgent=%u/%u us\n",
			  (unsigned long long)(snapshot.har_us / 1000ULL),
			  (unsigned long long)(snapshot.har_us % 1000ULL),
			  snapshot.accel_queue_drops,
			  snapshot.accel_late_samples,
			  snapshot.accel_max_late_us,
			  snapshot.urgent_events,
			  snapshot.urgent_last_block_us);
	} else {
		append_fmt(status_msg, sizeof(status_msg), &len, "waiting(%d/%d)\n",
			  (int)snapshot.har_sample_count, HAR_WINDOW_SAMPLES);
	}

	if (len >= sizeof(status_msg)) {
		len = sizeof(status_msg) - 1;
		status_msg[len] = '\0';
	}

	printk("%s", status_msg);
}

static void accel_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_accel_us = now_us();
	uint32_t accel_remainder_accum = 0U;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		uint64_t current_us;
		uint32_t late_us = 0U;

		sleep_until_us(next_accel_us);
		current_us = now_us();
		if (current_us > next_accel_us) {
			late_us = (uint32_t)MIN(current_us - next_accel_us, UINT32_MAX);
		}

		k_mutex_lock(&sensor_lock, K_FOREVER);
		task_accel_sample(ctx, current_us, late_us);
		k_mutex_unlock(&sensor_lock);
		accel_schedule_advance(&next_accel_us, &accel_remainder_accum);
	}
}

static void hts221_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = now_us() + HTS221_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		sleep_until_us(next_release_us);
		k_mutex_lock(&sensor_lock, K_FOREVER);
		task_read_hts221(ctx);
		k_mutex_unlock(&sensor_lock);

		do {
			next_release_us += ENV_PERIOD_US;
		} while (now_us() >= next_release_us);
	}
}

static void gyro_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = now_us();

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		sleep_until_us(next_release_us);
		k_mutex_lock(&sensor_lock, K_FOREVER);
		task_read_gyro(ctx);
		k_mutex_unlock(&sensor_lock);

		do {
			next_release_us += GYRO_PERIOD_US;
		} while (now_us() >= next_release_us);
	}
}

static void lis3mdl_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = now_us() + LIS3MDL_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		sleep_until_us(next_release_us);
		k_mutex_lock(&sensor_lock, K_FOREVER);
		task_read_lis3mdl(ctx);
		k_mutex_unlock(&sensor_lock);

		do {
			next_release_us += MAG_PERIOD_US;
		} while (now_us() >= next_release_us);
	}
}

static void lps22hb_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = now_us() + LPS22HB_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		sleep_until_us(next_release_us);
		k_mutex_lock(&sensor_lock, K_FOREVER);
		task_read_lps22hb(ctx);
		k_mutex_unlock(&sensor_lock);

		do {
			next_release_us += ENV_PERIOD_US;
		} while (now_us() >= next_release_us);
	}
}

static void har_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = now_us() + HAR_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		sleep_until_us(next_release_us);
		task_run_har_measure(ctx);

		do {
			next_release_us += HAR_PERIOD_US;
		} while (now_us() >= next_release_us);
	}
}

static void print_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = now_us() + PRINT_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		sleep_until_us(next_release_us);
		task_print_status(ctx);

		do {
			next_release_us += PRINT_PERIOD_US;
		} while (now_us() >= next_release_us);
	}
}

static void urgent_external_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		uint32_t rnd = pseudo_random32();
		uint32_t sleep_ms = URGENT_SLEEP_MIN_MS +
				    (rnd % (URGENT_SLEEP_MAX_MS - URGENT_SLEEP_MIN_MS + 1U));
		uint32_t block_us = URGENT_BLOCK_MIN_US +
				    (pseudo_random32() %
				     (URGENT_BLOCK_MAX_US - URGENT_BLOCK_MIN_US + 1U));

		k_sleep(K_MSEC(sleep_ms));

		k_mutex_lock(&app_lock, K_FOREVER);
		app.urgent_events++;
		app.urgent_last_block_us = block_us;
		k_mutex_unlock(&app_lock);

		k_busy_wait(block_us);
	}
}

static void start_threads(struct app_context *ctx)
{
	k_thread_create(&urgent_thread_data, urgent_stack, K_THREAD_STACK_SIZEOF(urgent_stack),
			urgent_external_thread, NULL, NULL, NULL,
			URGENT_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&accel_thread_data, accel_stack, K_THREAD_STACK_SIZEOF(accel_stack),
			accel_thread, ctx, NULL, NULL, ACCEL_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&gyro_thread_data, gyro_stack, K_THREAD_STACK_SIZEOF(gyro_stack),
			gyro_thread, ctx, NULL, NULL, GYRO_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&hts221_thread_data, hts221_stack, K_THREAD_STACK_SIZEOF(hts221_stack),
			hts221_thread, ctx, NULL, NULL, AUX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&lis3mdl_thread_data, lis3mdl_stack, K_THREAD_STACK_SIZEOF(lis3mdl_stack),
			lis3mdl_thread, ctx, NULL, NULL, AUX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&lps22hb_thread_data, lps22hb_stack, K_THREAD_STACK_SIZEOF(lps22hb_stack),
			lps22hb_thread, ctx, NULL, NULL, AUX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&har_thread_data, har_stack, K_THREAD_STACK_SIZEOF(har_stack),
			har_thread, ctx, NULL, NULL, HAR_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&print_thread_data, print_stack, K_THREAD_STACK_SIZEOF(print_stack),
			print_thread, ctx, NULL, NULL, PRINT_THREAD_PRIO, 0, K_NO_WAIT);
}

int main(void)
{
	struct sensor_value accel_odr_attr = { .val1 = 26, .val2 = 0 };
	struct sensor_value gyro_odr_attr = { .val1 = 12, .val2 = 0 };

	if (!device_is_ready(app.lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	if (!device_is_ready(app.hts221_dev) || !device_is_ready(app.lis3mdl_dev) ||
	    !device_is_ready(app.lps22hb_dev)) {
		printk("aux sensor: device not ready.\n");
		return 0;
	}

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
	start_threads(&app);

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
