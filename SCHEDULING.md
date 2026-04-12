# Scheduling Strategy

## Overview

This application now uses Zephyr preemptive threads instead of a single-threaded
cyclic executive. The goal is to keep accelerometer sampling independent from
slower work such as HAR inference and UART printing, and to inject a randomized
urgent external task that blocks normal execution for 30-50 ms.

The important design split is:

- the accelerometer thread samples at 26 Hz and writes samples into a bounded
  message queue
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

Accelerometer samples are queued in:

```c
K_MSGQ_DEFINE(accel_msgq, sizeof(struct accel_sample), 128, 4);
```

The queue acts as a cyclic software buffer. If the queue is full, the oldest
sample is removed and the newest sample is inserted. The status line reports:

- `qdrop`: queue overwrite count
- `late`: number of accelerometer releases that ran late
- `maxlate`: worst observed accelerometer lateness in microseconds
- `urgent`: urgent-event count and the last injected block length

The HAR thread drains all pending accelerometer samples before each inference,
so short inference or UART stalls do not cause backlog-driven sample loss.

## Hard Limit

A software queue only stores samples after the CPU has run the sampling thread.
If the highest-priority urgent event occupies the CPU for longer than the
accelerometer sample interval, the accelerometer thread cannot execute during
that interval. At 26 Hz, the interval is about 38.46 ms, so a 50 ms
highest-priority busy wait can still delay a sample.

To guarantee zero accelerometer data loss while a truly highest-priority task
blocks the CPU for 30-50 ms, the sampling path must use hardware assistance,
for example:

- LSM6DSL hardware FIFO with watermark/overrun handling
- a sensor-trigger interrupt path that records FIFO availability quickly
- DMA or another peripheral-side capture mechanism, if available

The current implementation is still useful for the assignment stress test
because it makes the scheduler behavior visible and prevents HAR inference,
UART output, and low-rate sensors from blocking each other. The `late` and
DRDY reports show whether the injected urgent task is exceeding what the
software-only design can absorb.
