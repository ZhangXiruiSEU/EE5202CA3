#include "app_threads.h"

#include <zephyr/sys/util.h>

#include "har_task.h"
#include "sensor_tasks.h"
#include "status.h"

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

static uint32_t pseudo_random32(void)
{
	static uint32_t state;

	if (state == 0U) {
		state = k_cycle_get_32() ^ 0xa5a55a5aU;
	}

	state = state * 1664525U + 1013904223U;
	return state;
}

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

static void accel_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_accel_us = app_now_us();
	uint32_t accel_remainder_accum = 0U;

	/* Zephyr thread entry points always receive three args; this thread only needs ctx. */
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		uint64_t current_us;
		uint32_t late_us = 0U;

		app_sleep_until_us(next_accel_us);
		current_us = app_now_us();
		if (current_us > next_accel_us) {
			late_us = (uint32_t)MIN(current_us - next_accel_us, UINT32_MAX);
		}

		task_accel_sample(ctx, current_us, late_us);
		accel_schedule_advance(&next_accel_us, &accel_remainder_accum);
	}
}

static void hts221_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = app_now_us() + HTS221_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		app_sleep_until_us(next_release_us);
		task_read_hts221(ctx);

		do {
			next_release_us += ENV_PERIOD_US;
		} while (app_now_us() >= next_release_us);
	}
}

static void gyro_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = app_now_us();

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		app_sleep_until_us(next_release_us);
		task_read_gyro(ctx);

		do {
			next_release_us += GYRO_PERIOD_US;
		} while (app_now_us() >= next_release_us);
	}
}

static void lis3mdl_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = app_now_us() + LIS3MDL_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		app_sleep_until_us(next_release_us);
		task_read_lis3mdl(ctx);

		do {
			next_release_us += MAG_PERIOD_US;
		} while (app_now_us() >= next_release_us);
	}
}

static void lps22hb_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = app_now_us() + LPS22HB_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		app_sleep_until_us(next_release_us);
		task_read_lps22hb(ctx);

		do {
			next_release_us += ENV_PERIOD_US;
		} while (app_now_us() >= next_release_us);
	}
}

static void har_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = app_now_us() + HAR_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		app_sleep_until_us(next_release_us);
		task_run_har_measure(ctx);

		do {
			next_release_us += HAR_PERIOD_US;
		} while (app_now_us() >= next_release_us);
	}
}

static void print_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;
	uint64_t next_release_us = app_now_us() + PRINT_PHASE_US;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		app_sleep_until_us(next_release_us);
		task_print_status(ctx);

		do {
			next_release_us += PRINT_PERIOD_US;
		} while (app_now_us() >= next_release_us);
	}
}

static void urgent_external_thread(void *p1, void *p2, void *p3)
{
	struct app_context *ctx = p1;

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
		ctx->urgent_events++;
		ctx->urgent_last_block_us = block_us;
		k_mutex_unlock(&app_lock);

		k_busy_wait(block_us);
	}
}

void app_threads_start(struct app_context *ctx)
{
	/* Start the highest-priority CPU-blocking stress task immediately. */
	k_thread_create(&urgent_thread_data, urgent_stack, K_THREAD_STACK_SIZEOF(urgent_stack),
			urgent_external_thread, ctx, NULL, NULL,
			URGENT_THREAD_PRIO, 0, K_NO_WAIT);

	/* Keep 26 Hz accelerometer sampling ahead of all normal application work. */
	k_thread_create(&accel_thread_data, accel_stack, K_THREAD_STACK_SIZEOF(accel_stack),
			accel_thread, ctx, NULL, NULL, ACCEL_THREAD_PRIO, 0, K_NO_WAIT);

	/* Sample gyroscope at the next highest normal sensor priority. */
	k_thread_create(&gyro_thread_data, gyro_stack, K_THREAD_STACK_SIZEOF(gyro_stack),
			gyro_thread, ctx, NULL, NULL, GYRO_THREAD_PRIO, 0, K_NO_WAIT);

	/* Low-rate environmental and magnetometer sensors share the auxiliary priority. */
	k_thread_create(&hts221_thread_data, hts221_stack, K_THREAD_STACK_SIZEOF(hts221_stack),
			hts221_thread, ctx, NULL, NULL, AUX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&lis3mdl_thread_data, lis3mdl_stack, K_THREAD_STACK_SIZEOF(lis3mdl_stack),
			lis3mdl_thread, ctx, NULL, NULL, AUX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&lps22hb_thread_data, lps22hb_stack, K_THREAD_STACK_SIZEOF(lps22hb_stack),
			lps22hb_thread, ctx, NULL, NULL, AUX_THREAD_PRIO, 0, K_NO_WAIT);

	/* Run HAR after samples have been buffered, below all sensor acquisition threads. */
	k_thread_create(&har_thread_data, har_stack, K_THREAD_STACK_SIZEOF(har_stack),
			har_thread, ctx, NULL, NULL, HAR_THREAD_PRIO, 0, K_NO_WAIT);
			
	/* Keep blocking UART output at the lowest application priority. */
	k_thread_create(&print_thread_data, print_stack, K_THREAD_STACK_SIZEOF(print_stack),
			print_thread, ctx, NULL, NULL, PRINT_THREAD_PRIO, 0, K_NO_WAIT);
}
