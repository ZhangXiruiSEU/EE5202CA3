#ifndef APPLICATIONS_LSM6DSL_LSM6DSL_FIFO_H_
#define APPLICATIONS_LSM6DSL_LSM6DSL_FIFO_H_

#include <zephyr/drivers/sensor.h>
#include <stdint.h>

int lsm6dsl_fifo_init(void);
int lsm6dsl_fifo_sample_count(uint16_t *sample_count);
int lsm6dsl_fifo_read_accel(struct sensor_value accel[3]);

#endif
