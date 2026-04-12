#ifndef APPLICATIONS_LSM6DSL_HAR_TASK_H_
#define APPLICATIONS_LSM6DSL_HAR_TASK_H_

#include "app_context.h"

int har_init(void);
void task_run_har_measure(struct app_context *ctx);

#endif
