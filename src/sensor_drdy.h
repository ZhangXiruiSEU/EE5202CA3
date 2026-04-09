#ifndef APPLICATIONS_LSM6DSL_SENSOR_DRDY_H_
#define APPLICATIONS_LSM6DSL_SENSOR_DRDY_H_

#include <zephyr/kernel.h>
#include <stddef.h>

int sensor_drdy_init(void);

void sensor_drdy_poll_accel(void);
void sensor_drdy_poll_gyro(void);
void sensor_drdy_poll_hts221(void);
void sensor_drdy_poll_lis3mdl(void);
void sensor_drdy_poll_lps22hb(void);

bool sensor_drdy_report_ready(uint64_t now_us);
void sensor_drdy_format_report(char *buf, size_t buf_size, uint64_t now_us);

#endif
