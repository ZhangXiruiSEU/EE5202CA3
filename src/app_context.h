#ifndef APPLICATIONS_LSM6DSL_APP_CONTEXT_H_
#define APPLICATIONS_LSM6DSL_APP_CONTEXT_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>

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
	/* Device handles resolved from devicetree at build time. */
	const struct device *lsm6dsl_dev;
	const struct device *hts221_dev;
	const struct device *lis3mdl_dev;
	const struct device *lps22hb_dev;

	/* Latest sensor values published by the sensor threads. */
	struct sensor_value accel[3];
	struct sensor_value humidity;
	struct sensor_value temperature;
	struct sensor_value pressure;
	struct sensor_value gyro[3];
	struct sensor_value magnetic[3];

	/* Latest HAR inference timing and output state. */
	uint32_t har_cycles;
	uint64_t har_us;
	int har_status;
	float har_scores[3];
	size_t har_sample_count;

	/* Accelerometer buffering and scheduling diagnostics. */
	uint32_t accel_queue_drops;
	uint32_t accel_late_samples;
	uint32_t accel_max_late_us;

	/* Urgent external task diagnostics. */
	uint32_t urgent_events;
	uint32_t urgent_last_block_us;
};

struct accel_sample {
	struct sensor_value accel[3];
	uint64_t timestamp_us;
	uint32_t late_us;
};

/* Shared kernel objects are defined in main.c; these declarations do not allocate them. */
extern struct k_mutex app_lock;
extern struct k_mutex sensor_lock;
extern struct k_msgq accel_msgq;

static inline uint64_t app_now_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

static inline void app_sleep_until_us(uint64_t target_us)
{
	uint64_t current_us = app_now_us();

	if (current_us < target_us) {
		k_sleep(K_USEC((int32_t)(target_us - current_us)));
	}
}

#endif
