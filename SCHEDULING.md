# Scheduling Strategy

## Overview

This application now uses Zephyr preemptive threads instead of a single-threaded
cyclic executive. The goal is to keep accelerometer sampling independent from
slower work such as HAR inference and UART printing, and to inject a randomized
urgent external task that blocks normal execution for 30-50 ms.

The important design split is:

- the accelerometer thread samples at 26 Hz and writes samples into a bounded
  message queue after draining the LSM6DSL hardware FIFO
- the HAR thread drains that queue at 1 Hz and maintains the 26-sample rolling
  model window
- low-rate sensors run in separate periodic threads
- UART status printing runs in its own low-priority thread
- the urgent external event runs at the highest priority and uses `k_busy_wait`
  to model CPU occupation

## Thread Priorities

Lower numeric priority is higher priority in Zephyr's preemptive scheduler.

| Thread | Priority | Role |
| --- | ---: | --- |
| urgent external event | 0 | random 30-50 ms CPU block |
| accelerometer | 1 | 26 Hz HAR input sampling |
| gyroscope | 2 | 12.5 Hz sampling |
| HTS221 / LIS3MDL / LPS22HB | 3 | low-rate auxiliary sensors |
| HAR inference | 4 | 1 Hz model run |
| UART print | 5 | 1 Hz status output |

The sensor threads share a `sensor_lock` so concurrent I2C and sensor driver
calls do not overlap. Shared application state is protected by `app_lock`.
DRDY accounting is protected inside `sensor_drdy.c`.

## Buffering

Accelerometer data now uses two buffering stages.

First, the LSM6DSL is programmed for accel-only FIFO stream mode at 26 Hz. If
the urgent external thread occupies the CPU for 30-50 ms, the sensor continues
placing accelerometer words in its own hardware FIFO. When the accelerometer
thread runs again, it reads the FIFO level and drains every complete accel
sample before returning to normal scheduling.

Second, drained samples are queued in:

```c
K_MSGQ_DEFINE(accel_msgq, sizeof(struct accel_sample), 128, 4);
```

This queue is the software handoff from the accel thread to the HAR thread. It
acts as a cyclic software buffer. If the queue is full, the oldest
sample is removed and the newest sample is inserted. The status line reports:

- `qdrop`: queue overwrite count
- `late`: number of accelerometer releases that ran late
- `maxlate`: worst observed accelerometer lateness in microseconds
- `urgent`: urgent-event count and the last injected block length

The HAR thread drains all pending accelerometer samples before each inference,
so short inference or UART stalls do not cause backlog-driven sample loss.

## Hardware FIFO

Zephyr's generic sensor API gives convenient `sensor_sample_fetch()` and
`sensor_channel_get()` calls, but it does not expose a portable "drain all FIFO
samples" operation for this sensor. The application therefore programs the
LSM6DSL FIFO registers directly through I2C in `lsm6dsl_fifo.c`, while still
using Zephyr's LSM6DSL driver for normal accel/gyro channel reads.

The FIFO configuration is:

- accel samples only
- no FIFO decimation
- 26 Hz FIFO output data rate
- stream mode, so newer samples keep arriving while the CPU is busy

The remaining hard limit is FIFO capacity and bus drain time. A 30-50 ms urgent
CPU block at 26 Hz is well within the intended use case. If the system were
blocked for much longer, or if the I2C bus were unavailable for too long, the
FIFO could eventually overrun and the software queue could still overwrite old
samples. The `late`, `qdrop`, and DRDY reports are kept to make those failure
modes visible during the stress test.
