#ifndef APPLICATIONS_LSM6DSL_SENSOR_TASKS_H_
#define APPLICATIONS_LSM6DSL_SENSOR_TASKS_H_

#include "app_context.h"

void task_accel_sample(struct app_context *ctx, uint64_t timestamp_us, uint32_t late_us);
void task_read_gyro(struct app_context *ctx);
void task_read_hts221(struct app_context *ctx);
void task_read_lis3mdl(struct app_context *ctx);
void task_read_lps22hb(struct app_context *ctx);

#endif
