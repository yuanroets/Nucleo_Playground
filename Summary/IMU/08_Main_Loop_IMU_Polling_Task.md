# 08 Main Loop IMU Polling Task

## Code area

The IMU block inside `while (1)` in `main.c`:

1. check elapsed time
2. call `bmi2_get_sensor_data`
3. print data or error

Ownership split:

1. [OURS]: scheduling and reaction logic.
2. [DRIVER]: Bosch API that performs transfer+parse.

---

## Why polling is used now

Polling is simplest way to learn first:

1. easy to reason about
2. easy to debug with UART
3. no interrupt routing complexity yet

---

## Step-by-step behavior

1. Compare `HAL_GetTick() - last_imu_tick` with 200 ms.
2. If interval elapsed, update `last_imu_tick`.
3. Call `bmi2_get_sensor_data(&bmi_sens_data, &bmi_dev)`.
4. On success, print raw `x/y/z` accel.
5. On failure, print error code.

---

## Timing pattern detail

`if ((HAL_GetTick() - last_imu_tick) >= 200U)`

Why this pattern is preferred:

1. non-blocking
2. allows other tasks in loop
3. works robustly with unsigned tick wrap-around

This is core embedded pattern you should master.

---

## What Bosch API does in this call (high level)

`bmi2_get_sensor_data`:

1. reads needed sensor registers via callbacks,
2. parses raw bytes into structured fields,
3. fills `bmi_sens_data`.

You do not need its internal parser details right now.

---

## Why data is called "raw"

Current print uses integer counts directly from driver output fields.

At this stage raw is enough to confirm:

1. communication is alive,
2. orientation changes are visible,
3. signs/magnitudes look reasonable.

Later you can convert to g or m/s^2 for physical units.

---

## Common beginner mistakes here

1. Forgetting to update `last_imu_tick`.
2. Using blocking `HAL_Delay` in loop and starving other tasks.
3. Not handling read errors visibly.

---

## What you must master

1. Non-blocking periodic task pattern.
2. Separation between scheduler logic (ours) and sensor API call (driver).

## What can be black box

1. Bosch parse internals behind `bmi2_get_sensor_data`.
