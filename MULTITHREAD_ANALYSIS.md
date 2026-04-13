# Multithreaded Runtime Analysis


## Current Architecture

The application now uses multiple Zephyr threads instead of one cyclic
superloop. Sensor acquisition, HAR inference, status printing, and the
urgent load are separated into their own kernel threads.

For more background on how this Zephyr-based project differs from the default
STM32CubeIDE workflow, as well as the Zephyr development environment setup, see the ***Appendix A - Zephyr vs STM32Cube IDE***.

## Current Thread Model

The runtime is split into independent Zephyr threads. Each thread sleeps until
its next planned release time, performs one bounded unit of work, then advances
its own schedule.

Thread priorities are defined in `src/app_context.h`. Lower numeric priority
means higher scheduling priority in Zephyr.

| Thread | Period / phase | Priority | Main function |
| --- | --- | ---: | --- |
| urgent external task | random sleep, then `30-50 ms` busy wait | `0` | Creates high-priority CPU interference for stress testing. |
| accel | `104 / 4 s`, about `38.46 ms` | `1` | Polls accel DRDY, drains LSM6DSL FIFO if available, and pushes accel samples into `accel_msgq`. |
| gyro | `80 ms` | `2` | Polls gyro DRDY and updates the latest gyro value. |
| HTS221 | `1000 ms`, `200 ms` phase | `3` | Reads humidity and temperature. |
| LPS22HB | `1000 ms`, `400 ms` phase | `3` | Reads pressure. |
| LIS3MDL | `800 ms`, `600 ms` phase | `3` | Reads magnetic field. |
| HAR | `1000 ms`, `900 ms` phase | `4` | Drains accel queue, updates the rolling HAR window, and runs inference. |
| print | `1000 ms`, `950 ms` phase | `5` | Prints either a DRDY report or the latest status snapshot. |

The practical priority order is:

```text
urgent stress > accel > gyro > aux sensors > HAR > print
```

The urgent thread is intentionally above accel. It exists to simulate external
high-priority CPU interference and to test whether FIFO and queueing can keep
the HAR pipeline alive under stress.

Sensor bus access is serialized by `sensor_lock`, so two sensor threads do not
talk to the shared buses at the same time. Application state is protected by
`app_lock`, so the print thread can copy a consistent snapshot while sensor and
HAR threads are updating values.

Both locks are Zephyr mutexes:

```c
K_MUTEX_DEFINE(app_lock);
K_MUTEX_DEFINE(sensor_lock);
```

Sensor reads take `sensor_lock` around the bus transaction:

```c
k_mutex_lock(&sensor_lock, K_FOREVER);
ret = sensor_sample_fetch(ctx->hts221_dev);
ret = sensor_channel_get(ctx->hts221_dev, SENSOR_CHAN_HUMIDITY, &humidity);
k_mutex_unlock(&sensor_lock);
```

Shared application state updates take `app_lock`:

```c
k_mutex_lock(&app_lock, K_FOREVER);
ctx->humidity = humidity;
ctx->temperature = temperature;
k_mutex_unlock(&app_lock);
```

The important design choice is that acquisition threads run above HAR and
printing. UART output and inference may be delayed, but they should not block
the normal sensor read path unless a higher-priority thread is already using
the CPU.

## Sensor Data Flow In This Project

The data path is:

```text
Zephyr devicetree device
  -> Zephyr sensor driver
  -> sensor task thread
  -> app_context latest-value fields
  -> status print thread
```

For accelerometer data, there is an extra buffering path because HAR depends on
the full sample sequence:

```text
LSM6DSL accel hardware
  -> DRDY poll / FIFO drain
  -> accel_msgq
  -> HAR rolling window, 26 x 3 samples
  -> HAR network inference
  -> app_context HAR result
  -> status print thread
```

The accel thread first calls `sensor_drdy_poll_accel()`, then checks the
LSM6DSL FIFO. If the FIFO contains samples, it reads all pending accel samples
and enqueues them.  A direct
`sensor_sample_fetch_chan(..., SENSOR_CHAN_ACCEL_XYZ)` read is now only used as
an emergency fallback if FIFO status cannot be read.

Other sensors use a simpler latest-value model:

```text
gyro     -> ctx->gyro
HTS221   -> ctx->humidity / ctx->temperature
LIS3MDL  -> ctx->magnetic
LPS22HB  -> ctx->pressure
```

These values are not queued. Each sensor thread overwrites the latest value in
`app_context`, and the print thread later copies the current snapshot. That is
acceptable here because these sensors are low-rate status inputs rather than
the ordered HAR input stream.

DRDY accounting is separate from the data storage path. Each sensor task polls
its DRDY status before fetching data, and `sensor_drdy_format_report()` later
summarizes those counters in the print thread.

## Sensor Timing

Current planned rates:

| Sensor / task | Period | Expected count per 4 s |
| --- | ---: | ---: |
| LSM6DSL accel | `104 samples / 4 s` | `104` |
| LSM6DSL gyro | `80 ms` | `50` |
| HTS221 humidity / temperature | `1000 ms` | `4` |
| LIS3MDL magnetometer | `800 ms` | `5` |
| LPS22HB pressure | `1000 ms` | `4` |
| HAR inference | `1000 ms` | `4` |
| status print | `1000 ms` | `4` |

Accel keeps the fractional `26 Hz` schedule with integer remainder
accumulation:

```text
4,000,000 us / 104 = 38,461.538461... us
```

The thread adds `38,461 us` each sample and carries the remainder so the
long-term 4-second cadence stays exact.

Gyro, HTS221, LIS3MDL, and LPS22HB use strict periodic thread loops. This is
important because their DRDY bits follow the sensor hardware ODR.

HAR and print are lower priority and can be delayed without corrupting the
sensor sampling cadence.

## Accel FIFO And Queue

The accelerometer path uses two layers of buffering:

1. LSM6DSL hardware FIFO for accel samples.
2. Zephyr `accel_msgq` from the accel thread to the HAR thread.

The accel thread first tries to drain pending FIFO samples. If FIFO is empty,
it skips that cycle instead of reading the current accel output registers. 

If the hardware FIFO produces slightly more than 104 accel samples in one
4-second software window, the extra samples are still drained from FIFO but are
not accepted into `accel_msgq`. This keeps the HAR input stream fixed at the
model's expected 26 Hz rate.

This can happen because the software schedule and the sensor FIFO are driven by
different clocks. The application defines the HAR input cadence as exactly
`104` samples per `4 s`, using Zephyr uptime:

```text
4,000,000 us / 104 = 38,461.538461... us
```

The LSM6DSL FIFO uses the sensor's internal ODR setting. Its `26 Hz` setting is
a nominal sensor rate, not a clock locked to Zephyr's software timer. In
practice this can produce slightly more samples, such as `106` or `107` FIFO
samples in one software 4-second window.

The current policy is therefore accepting at most 104 samples into accel_msgq per 4-second window.

This is why a single accel-thread delay does not necessarily mean HAR input was
lost. If the hardware FIFO has the missed sample, the accel thread can recover
it later.

The HAR thread drains `accel_msgq` and pushes samples into the `26 x 3` HAR
window.

## Status Diagnostics

Status lines include:

```text
qdrop=0 late=46980 maxlate=49624 us urgent=2937/41877 us
```

Meaning:

| Field | Meaning |
| --- | --- |
| `qdrop` | Number of accel samples dropped because `accel_msgq` was full. `0` means no accepted accel sample was pushed out of the software queue before HAR could consume it. |
| `late` | Cumulative number of accel thread wakeups that occurred after their planned time. Even tiny lateness increments this. |
| `maxlate` | Largest observed accel scheduling lateness in microseconds. |
| `urgent=A/B` | `A` is urgent thread event count; `B` is the most recent urgent busy-wait duration in microseconds. |

Important interpretation:

- `late` is not a lost-sample counter.
- `maxlate` shows worst scheduling interference.
- `qdrop=0` is the stronger sign that the software accel queue is not
  overflowing.
- If FIFO is working and `qdrop=0`, occasional accel lateness can be tolerated.

## DRDY Report Interpretation

The project includes a custom DRDY diagnostic module:

```text
src/sensor_drdy.c
src/sensor_drdy.h
```

It reads sensor status registers through Zephyr I2C and summarizes whether data
was ready when each sensor task polled its device.

The DRDY report prints lines like:

```text
drdy4s a=104/104 d0 (0.00%) accel_in=104 g=50/50 d0 (0.00%) h=4/4 d0 (0.00%) m=5/5 d0 (0.00%) p=4/4 d0 (0.00%)
```

Field meaning:

| Symbol | Sensor / channel |
| --- | --- |
| `a` | LSM6DSL accelerometer |
| `g` | LSM6DSL gyroscope |
| `h` | HTS221 humidity / temperature |
| `m` | LIS3MDL magnetometer |
| `p` | LPS22HB pressure |

Counter meaning:

| Field | Meaning |
| --- | --- |
| `drdy4s` | The report covers one fixed 4-second DRDY accounting window. |
| `a=104/104` | Accelerometer DRDY ready-count. Because accel data is recovered through FIFO, this is no longer the best missing-sample indicator for HAR input. |
| `accel_in=N` | Number of accelerometer samples accepted into the HAR input queue during this 4-second window. For accel, this is the main value to check; it is capped at `104` to keep the HAR input rate at 26 Hz. |
| `g=50/50`, `h=4/4`, `m=5/5`, `p=4/4` | Ready-count compared with the expected count for non-FIFO sensor reads in 4 seconds. These counters are still useful as missing-sample indicators for those sensors. |
| `dN` | Duplicate / not-ready count. DRDY was not set when polled. |

For accelerometer data, FIFO changes the interpretation. The `a` counter only
shows whether the accel DRDY bit was set when the accel thread polled the
status register. Since the thread may read multiple buffered FIFO samples after
one DRDY poll, `a` does not equal the number of accel samples delivered to HAR.
Use `accel_in=104` to check whether the 4-second HAR input window received the
expected number of accelerometer samples.

For gyro, HTS221, LIS3MDL, and LPS22HB, there is no equivalent FIFO recovery in
the current code path, so their ready-count fields are still the useful strict
per-window missing indicators.

Example:

```text
g=49/50 d1
```

This means the 4-second window captured 49 ready gyro events out of the
expected 50. The `d1` means one gyro poll happened when the gyro DRDY bit was
not set, so that window missed one fresh gyro sample opportunity. For strict
per-window accounting, this should be counted as one gyro miss.



## Current Observations

Recent logs show mostly stable windows:

```text
drdy4s a=104/104 d0 (0.00%) accel_in=104 g=50/50 d0 (0.00%) h=4/4 d0 (0.00%) m=5/5 d0 (0.00%) p=4/4 d0 (0.00%)
drdy4s a=104/104 d0 (0.00%) accel_in=104 g=48/50 d2 (4.00%) h=4/4 d0 (0.00%) m=5/5 d0 (0.00%) p=4/4 d0 (0.00%)
```

The strongest current signal is:

```text
a=104/104 d0
accel_in=104
h=4/4 d0
m=5/5 d0
p=4/4 d0
```

This indicates the accelerometer HAR input, humidity/temperature,
magnetometer, and pressure ready-counts are stable in normal 4-second windows.

Using the current missing-data definition:

- accelerometer missing count is `104 - accel_in`;
- gyro, HTS221, LIS3MDL, and LPS22HB missing counts are their `dN` values.

For the sampled log set with 11 `drdy4s` windows:

| Sensor | Expected per window | Total expected | Total missing | Missing rate |
| --- | ---: | ---: | ---: | ---: |
| accelerometer | `104` | `1144` | `0` | `0.00%` |
| gyroscope | `50` | `550` | `12` | `2.18%` |
| HTS221 humidity / temperature | `4` | `44` | `0` | `0.00%` |
| LIS3MDL magnetometer | `5` | `55` | `0` | `0.00%` |
| LPS22HB pressure | `4` | `44` | `0` | `0.00%` |

This shows that the accelerometer HAR input is complete in that log set:
`accel_in` is `104` in every 4-second window. The only observed missing samples
are gyro samples.

Gyro occasionally shows:

```text
g=49/50 d1
g=48/50 d2
```

This is plausible under the current stress load because:

- gyro period is `80 ms`;
- urgent thread can block CPU for `30-50 ms`;
- gyro DRDY is checked before fetching;
- a poll near the DRDY boundary can see not-ready and increment duplicate.


## Current Assessment

The current multithreaded design is behaving reasonably:

- HAR queue drops are `0`.
- Accel DRDY is usually `104/104`, and the more important HAR input counter is
  now stable at `accel_in=104`.
- Pressure and magnetometer ready-counts are stable in normal windows.
- Occasional gyro duplicate events are expected under high-priority stress.


The original assumption was that the accelerometer was the only sensor that
needed special protection from urgent high-priority interference, because accel
is the HAR input stream and has the highest data rate. That is why the current
implementation added LSM6DSL accel FIFO draining first.

The current logs show that gyro can also be affected. Its `80 ms` period is
slower than accel but still frequent enough that a `30-50 ms` urgent CPU block
can push a poll close to, or past, the next DRDY boundary. In strict 4-second
accounting, `g=49/50 d1` should therefore be treated as a missed fresh gyro
sample opportunity.

The best future improvement is to use hardware buffering wherever the sensor
supports it, instead of only using FIFO for accelerometer samples. For example,
gyro should also be recovered from the LSM6DSL FIFO if its FIFO stream is
enabled and configured correctly. 
