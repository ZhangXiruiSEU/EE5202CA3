# Scheduling Strategy

## Overview

This application uses a single-threaded cyclic executive in `src/main.c`.
All tasks run in the main loop. There is no separate sampling thread, inference
thread, or asynchronous UART path.

The scheduler is time-triggered:

- accelerometer sampling is released at `26 Hz`
- gyroscope sampling is released at `12.5 Hz`
- magnetometer sampling is released at `1.25 Hz`
- humidity / temperature / pressure sampling is released at `1 Hz`
- HAR inference is released at `1 Hz`
- UART status output is released at `1 Hz`

Because the scheduler is single-threaded, any blocking task delays following
tasks in the same time window. The main source of blocking at the moment is
UART console output.

## Hardware / Runtime Rates

The current design targets the following rates:

| Item | Target Rate | Notes |
| --- | --- | --- |
| LSM6DSL accelerometer | 26 Hz | Matches the HAR model input window |
| LSM6DSL gyroscope | 12.5 Hz | Lowest hardware-supported gyro ODR used here |
| HTS221 humidity | 1 Hz | Hardware ODR |
| HTS221 temperature | 1 Hz | Same device / same fetch as humidity |
| LIS3MDL magnetometer | 1.25 Hz | Hardware ODR |
| LPS22HB pressure | 1 Hz | Hardware ODR |
| HAR inference | 1 Hz | Uses the latest 26 accel samples |
| UART status message | 1 Hz | Concise status line |

## Schedule Table

The implementation uses absolute release times instead of a fixed minor-cycle
grid.

### Base periods

- accelerometer: exact `26 Hz` pacing via a fractional interval accumulator
  over a `4 s` major cycle
- gyroscope: every `80 ms`
- magnetometer: every `800 ms`
- HTS221: every `1000 ms`
- LPS22HB: every `1000 ms`
- HAR inference: `4` planned releases per `4 s` major cycle
- UART print: `4` planned releases per `4 s` major cycle

### Ordinary-task table

Gyro, HTS221, LPS22HB, and LIS3MDL are scheduled as strict periodic tasks so
their reads stay aligned with the hardware ODR and DRDY behavior. HAR and UART
print use an explicit 4-second release table in `src/main.c`, so the longer
software-only work can be placed inside accelerometer gaps.

| Task | Releases per 4 s | Placement |
| --- | ---: | --- |
| Accelerometer | 104 | Fractional 26 Hz accumulator |
| Gyroscope | 50 | Strict 80 ms periodic releases |
| HTS221 | 4 | Strict 1 s periodic releases |
| LPS22HB | 4 | Strict 1 s periodic releases |
| LIS3MDL | 5 | Strict 800 ms periodic releases |
| HAR inference | 4 | Centered in accel gaps with about 5 ms reserved |
| UART print | 4 | Centered in accel gaps with about 15 ms reserved |

This places the UART print after the HAR run so the status message contains the
latest classification result and inference time. HAR is planned as an about
`5 ms` task, and UART print is planned as an about `15 ms` task; both are placed
inside accel gaps with margin before the next accel release.

The accelerometer entry starts at `0 ms`, but unlike the other tasks it is not
released by a fixed integer period. It uses a fractional interval accumulator
to realize an exact `26 Hz` average rate over the `4 s` accelerometer major
cycle.

If an accelerometer release and one or more low-rate task releases happen at
the same scheduler instant, the main loop executes the accelerometer sample
first and only then runs the other ready tasks. This priority comes from the
execution order in `src/main.c`, not from the phase-offset table itself.

## HAR Input Strategy

- HAR input window size: `26 x 3`
- data source: accelerometer only
- input unit sent to the model: `mg / 4000.0`
- window policy: latest 26 accelerometer samples
- inference trigger: periodic at `1 Hz`, not immediate-on-fill

This means the model consumes one second of accelerometer history each time it
runs.

## Measured Timing

The following values were measured on the current build:

### HAR inference

- measured inference time: about `4.068 ms` per run

### UART status line

Measured result:

```text
uart msg=224B | print_cpu=19380 us | uart_line=19444 us
```

Interpretation:

- status message length: `224 bytes`
- CPU blocked in `printk("%s", status_msg)`: `19.380 ms`
- theoretical line time at `115200 baud`: `19.444 ms`

This means the current UART console path behaves almost like a synchronous,
blocking transmit path. CPU occupancy and UART line time are nearly identical.

## Scheduling Implication

The accelerometer release interval is about:

- `1000 / 26 = 38.46 ms`

Compared with the measured UART blocking time:

- UART print blocking: `19.38 ms`
- accelerometer interval: `38.46 ms`

So the current UART print is shorter than one accelerometer interval and should
not normally skip a full accel release. However, it still introduces timing
jitter because the scheduler is single-threaded and non-preemptive at the
application level.

Practical conclusion:

- current HAR time is safe for this schedule
- current UART print time is also acceptable
- UART output is the main source of jitter
- if the status message grows much larger, UART may start to interfere with the
  `26 Hz` accelerometer pacing

## Notes

- The UART timing above measures the main status line only.
- Any additional debug print line adds extra blocking time.
- If tighter timing is required, the next improvement should be reducing UART
  output size, increasing baud rate, or switching to an asynchronous UART path.
