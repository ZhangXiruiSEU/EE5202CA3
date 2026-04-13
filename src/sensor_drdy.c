#include "sensor_drdy.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdarg.h>
#include <string.h>

#include "app_context.h"

/* Keep DRDY accounting aligned with the app's 4 s major cycle. */
#define DRDY_WINDOW_US 4000000ULL

#define LSM6DSL_REG_STATUS_REG 0x1E
#define LSM6DSL_STATUS_GDA BIT(1)
#define LSM6DSL_STATUS_XLDA BIT(0)

#define HTS221_REG_STATUS 0x27
#define HTS221_STATUS_H_DA BIT(1)
#define HTS221_STATUS_T_DA BIT(0)

#define LIS3MDL_REG_STATUS 0x27
#define LIS3MDL_STATUS_ZYXOR BIT(7)
#define LIS3MDL_STATUS_ZYXDA BIT(3)

#define LPS22HB_REG_STATUS 0x27
#define LPS22HB_STATUS_P_OR BIT(5)
#define LPS22HB_STATUS_P_DA BIT(1)

#define ACCEL_EXPECTED_PER_WINDOW 104U
#define GYRO_EXPECTED_PER_WINDOW 50U
#define HTS221_EXPECTED_PER_WINDOW 4U
#define LIS3MDL_EXPECTED_PER_WINDOW 5U
#define LPS22HB_EXPECTED_PER_WINDOW 4U

struct drdy_counter {
	uint32_t captured;
	uint32_t duplicates;
	uint32_t overruns;
};

struct drdy_state {
	struct drdy_counter accel;
	struct drdy_counter gyro;
	struct drdy_counter hts221;
	struct drdy_counter lis3mdl;
	struct drdy_counter lps22hb;
	struct drdy_counter report_accel;
	struct drdy_counter report_gyro;
	struct drdy_counter report_hts221;
	struct drdy_counter report_lis3mdl;
	struct drdy_counter report_lps22hb;
	uint32_t accel_in;
	uint32_t report_accel_in;
	uint64_t window_start_us;
	uint64_t report_window_start_us;
	bool report_pending;
	bool initialized;
};

static const struct i2c_dt_spec lsm6dsl_i2c = I2C_DT_SPEC_GET(DT_INST(0, st_lsm6dsl));
static const struct i2c_dt_spec hts221_i2c = I2C_DT_SPEC_GET(DT_INST(0, st_hts221));
static const struct i2c_dt_spec lis3mdl_i2c = I2C_DT_SPEC_GET(DT_INST(0, st_lis3mdl_magn));
static const struct i2c_dt_spec lps22hb_i2c = I2C_DT_SPEC_GET(DT_INST(0, st_lps22hb_press));

static struct drdy_state drdy_state;
K_MUTEX_DEFINE(drdy_lock);

static void drdy_append_fmt(char *buf, size_t buf_size, size_t *offset, const char *fmt, ...)
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

static int read_status_reg(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *value)
{
	int ret;

	k_mutex_lock(&sensor_lock, K_FOREVER);
	ret = i2c_reg_read_byte_dt(spec, reg, value);
	k_mutex_unlock(&sensor_lock);

	return ret;
}

static void update_counter(struct drdy_counter *counter, bool data_ready, bool overrun)
{
	/* A missing DRDY bit means this poll likely reread the previous sample. */
	if (data_ready) {
		counter->captured++;
	} else {
		counter->duplicates++;
	}

	if (overrun) {
		counter->overruns++;
	}
}

static void reset_current_counters(void)
{
	(void)memset(&drdy_state.accel, 0, sizeof(drdy_state.accel));
	(void)memset(&drdy_state.gyro, 0, sizeof(drdy_state.gyro));
	(void)memset(&drdy_state.hts221, 0, sizeof(drdy_state.hts221));
	(void)memset(&drdy_state.lis3mdl, 0, sizeof(drdy_state.lis3mdl));
	(void)memset(&drdy_state.lps22hb, 0, sizeof(drdy_state.lps22hb));
	drdy_state.accel_in = 0U;
}

static void close_elapsed_windows_locked(uint64_t now_us)
{
	if (!drdy_state.initialized ||
	    (now_us - drdy_state.window_start_us) < DRDY_WINDOW_US) {
		return;
	}

	/*
	 * Keep the report semantics fixed at 4 s. If the low-priority print
	 * thread is late, counters after the boundary belong to the next
	 * window instead of stretching the completed report to 5 s.
	 */
	if (!drdy_state.report_pending) {
		drdy_state.report_accel = drdy_state.accel;
		drdy_state.report_gyro = drdy_state.gyro;
		drdy_state.report_hts221 = drdy_state.hts221;
		drdy_state.report_lis3mdl = drdy_state.lis3mdl;
		drdy_state.report_lps22hb = drdy_state.lps22hb;
		drdy_state.report_accel_in = drdy_state.accel_in;
		drdy_state.report_window_start_us = drdy_state.window_start_us;
		drdy_state.report_pending = true;
	}

	reset_current_counters();

	do {
		drdy_state.window_start_us += DRDY_WINDOW_US;
	} while ((now_us - drdy_state.window_start_us) >= DRDY_WINDOW_US);
}

static uint32_t percent_x100(uint32_t captured, uint32_t expected)
{
	uint32_t missed = 0U;

	if (captured < expected) {
		missed = expected - captured;
	}

	return (uint32_t)(((uint64_t)missed * 10000ULL) / expected);
}

static void append_counter(char *buf, size_t buf_size, size_t *offset, const char *name,
			  const struct drdy_counter *counter, uint32_t expected)
{
	uint32_t miss_x100 = percent_x100(counter->captured, expected);

	if (*offset >= buf_size) {
		return;
	}

	drdy_append_fmt(buf, buf_size, offset, "%s=%u/%u d%u (%u.%02u%%)",
		       name,
		       counter->captured,
		       expected,
		       counter->duplicates,
		       miss_x100 / 100U,
		       miss_x100 % 100U);
}

int sensor_drdy_init(void)
{
	if (!i2c_is_ready_dt(&lsm6dsl_i2c) ||
	    !i2c_is_ready_dt(&hts221_i2c) ||
	    !i2c_is_ready_dt(&lis3mdl_i2c) ||
	    !i2c_is_ready_dt(&lps22hb_i2c)) {
		return -ENODEV;
	}

	k_mutex_lock(&drdy_lock, K_FOREVER);
	drdy_state.window_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
	drdy_state.initialized = true;
	k_mutex_unlock(&drdy_lock);
	return 0;
}

void sensor_drdy_poll_accel(void)
{
	uint8_t status;
	int ret;

	ret = read_status_reg(&lsm6dsl_i2c, LSM6DSL_REG_STATUS_REG, &status);
	if (ret < 0) {
		return;
	}

	k_mutex_lock(&drdy_lock, K_FOREVER);
	if (!drdy_state.initialized) {
		k_mutex_unlock(&drdy_lock);
		return;
	}

	close_elapsed_windows_locked(k_ticks_to_us_floor64(k_uptime_ticks()));
	update_counter(&drdy_state.accel, (status & LSM6DSL_STATUS_XLDA) != 0U, false);
	k_mutex_unlock(&drdy_lock);
}

void sensor_drdy_poll_gyro(void)
{
	uint8_t status;
	int ret;

	ret = read_status_reg(&lsm6dsl_i2c, LSM6DSL_REG_STATUS_REG, &status);
	if (ret < 0) {
		return;
	}

	k_mutex_lock(&drdy_lock, K_FOREVER);
	if (!drdy_state.initialized) {
		k_mutex_unlock(&drdy_lock);
		return;
	}

	close_elapsed_windows_locked(k_ticks_to_us_floor64(k_uptime_ticks()));
	update_counter(&drdy_state.gyro, (status & LSM6DSL_STATUS_GDA) != 0U, false);
	k_mutex_unlock(&drdy_lock);
}

void sensor_drdy_poll_hts221(void)
{
	uint8_t status;
	bool ready;
	int ret;

	ret = read_status_reg(&hts221_i2c, HTS221_REG_STATUS, &status);
	if (ret < 0) {
		return;
	}

	k_mutex_lock(&drdy_lock, K_FOREVER);
	if (!drdy_state.initialized) {
		k_mutex_unlock(&drdy_lock);
		return;
	}

	close_elapsed_windows_locked(k_ticks_to_us_floor64(k_uptime_ticks()));
	ready = (status & (HTS221_STATUS_H_DA | HTS221_STATUS_T_DA)) != 0U;
	update_counter(&drdy_state.hts221, ready, false);
	k_mutex_unlock(&drdy_lock);
}

void sensor_drdy_poll_lis3mdl(void)
{
	uint8_t status;
	int ret;

	ret = read_status_reg(&lis3mdl_i2c, LIS3MDL_REG_STATUS, &status);
	if (ret < 0) {
		return;
	}

	k_mutex_lock(&drdy_lock, K_FOREVER);
	if (!drdy_state.initialized) {
		k_mutex_unlock(&drdy_lock);
		return;
	}

	close_elapsed_windows_locked(k_ticks_to_us_floor64(k_uptime_ticks()));
	update_counter(&drdy_state.lis3mdl,
		      (status & LIS3MDL_STATUS_ZYXDA) != 0U,
		      (status & LIS3MDL_STATUS_ZYXOR) != 0U);
	k_mutex_unlock(&drdy_lock);
}

void sensor_drdy_poll_lps22hb(void)
{
	uint8_t status;
	int ret;

	ret = read_status_reg(&lps22hb_i2c, LPS22HB_REG_STATUS, &status);
	if (ret < 0) {
		return;
	}

	k_mutex_lock(&drdy_lock, K_FOREVER);
	if (!drdy_state.initialized) {
		k_mutex_unlock(&drdy_lock);
		return;
	}

	close_elapsed_windows_locked(k_ticks_to_us_floor64(k_uptime_ticks()));
	update_counter(&drdy_state.lps22hb,
		      (status & LPS22HB_STATUS_P_DA) != 0U,
		      (status & LPS22HB_STATUS_P_OR) != 0U);
	k_mutex_unlock(&drdy_lock);
}

bool sensor_drdy_reserve_accel_in(uint64_t sample_us)
{
	bool accepted = false;

	k_mutex_lock(&drdy_lock, K_FOREVER);
	if (!drdy_state.initialized) {
		k_mutex_unlock(&drdy_lock);
		return false;
	}

	if (sample_us < drdy_state.window_start_us) {
		if (drdy_state.report_pending &&
		    sample_us >= drdy_state.report_window_start_us &&
		    sample_us < (drdy_state.report_window_start_us + DRDY_WINDOW_US) &&
		    drdy_state.report_accel_in < ACCEL_EXPECTED_PER_WINDOW) {
			drdy_state.report_accel_in++;
			accepted = true;
		}
		k_mutex_unlock(&drdy_lock);
		return accepted;
	}

	close_elapsed_windows_locked(sample_us);
	if (drdy_state.accel_in < ACCEL_EXPECTED_PER_WINDOW) {
		drdy_state.accel_in++;
		accepted = true;
	}
	k_mutex_unlock(&drdy_lock);
	return accepted;
}

bool sensor_drdy_report_ready(uint64_t now_us)
{
	bool ready;

	k_mutex_lock(&drdy_lock, K_FOREVER);
	close_elapsed_windows_locked(now_us);
	ready = drdy_state.initialized && drdy_state.report_pending;
	k_mutex_unlock(&drdy_lock);

	return ready;
}

void sensor_drdy_format_report(char *buf, size_t buf_size, uint64_t now_us)
{
	size_t len = 0;

	if (buf_size == 0U) {
		return;
	}

	buf[0] = '\0';
	k_mutex_lock(&drdy_lock, K_FOREVER);
	close_elapsed_windows_locked(now_us);
	if (!drdy_state.initialized || !drdy_state.report_pending) {
		k_mutex_unlock(&drdy_lock);
		return;
	}

	/* Compact report format keeps UART output short enough for 1 Hz printing. */
	drdy_append_fmt(buf, buf_size, &len, "drdy4s ");
	append_counter(buf, buf_size, &len, "a", &drdy_state.report_accel,
		      ACCEL_EXPECTED_PER_WINDOW);
	drdy_append_fmt(buf, buf_size, &len, " accel_in=%u", drdy_state.report_accel_in);
	drdy_append_fmt(buf, buf_size, &len, " ");
	append_counter(buf, buf_size, &len, "g", &drdy_state.report_gyro,
		      GYRO_EXPECTED_PER_WINDOW);
	drdy_append_fmt(buf, buf_size, &len, " ");
	append_counter(buf, buf_size, &len, "h", &drdy_state.report_hts221,
		      HTS221_EXPECTED_PER_WINDOW);
	drdy_append_fmt(buf, buf_size, &len, " ");
	append_counter(buf, buf_size, &len, "m", &drdy_state.report_lis3mdl,
		      LIS3MDL_EXPECTED_PER_WINDOW);
	drdy_append_fmt(buf, buf_size, &len, " ");
	append_counter(buf, buf_size, &len, "p", &drdy_state.report_lps22hb,
		      LPS22HB_EXPECTED_PER_WINDOW);
	drdy_append_fmt(buf, buf_size, &len, "\n");

	if (len >= buf_size) {
		buf[buf_size - 1U] = '\0';
	}

	drdy_state.report_pending = false;
	k_mutex_unlock(&drdy_lock);
}
