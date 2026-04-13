#include "status.h"

#include <zephyr/sys/printk.h>
#include <stdarg.h>

#include "sensor_drdy.h"

static char status_msg[STATUS_MSG_SIZE];
static char drdy_msg[192];

static const char *const har_labels[3] = {
	"stationary",
	"walking",
	"running",
};

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

void task_print_status(struct app_context *ctx)
{
	size_t len = 0;
	struct app_context snapshot;

	sensor_drdy_format_report(drdy_msg, sizeof(drdy_msg), app_now_us());
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
