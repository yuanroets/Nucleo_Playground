# 01 IMU Global State And Context

## Why this exists

Before function calls make sense, you need to understand the data objects they read and write.

Relevant code area in `Core/Src/main.c`:

1. `struct bmi2_dev bmi_dev;`
2. `struct bmi2_sens_data bmi_sens_data = {{0}};`
3. `typedef struct { I2C_HandleTypeDef *hi2c; uint16_t dev_addr; } bmi2_i2c_ctx_t;`
4. `bmi2_i2c_ctx_t bmi2_i2c_ctx;`
5. `uint32_t last_imu_tick = 0;`

---

## Big picture mental model

Think of IMU integration as three layers of state:

1. Application state (timers/logic in your main loop).
2. Interface state (which bus/address to talk to).
3. Driver state (Bosch object that tracks how to use sensor).

---

## 1) `bmi_dev` (Bosch driver object)

Ownership: [DRIVER object configured by OUR code]

What it stores:

1. Which interface is used (I2C or SPI).
2. Function pointers (read/write/delay callbacks).
3. Context pointer (`intf_ptr`) passed back to callbacks.
4. Internal fields Bosch uses (chip id, status flags, etc.).

Why it matters:

1. Almost every Bosch API needs `&bmi_dev`.
2. If callbacks or context are wrong, all Bosch calls fail.

Think of it as a "session object" for one sensor instance.

---

## 2) `bmi_sens_data` (latest sample buffer)

Ownership: [OURS owns variable, DRIVER fills it]

What it is:

1. A struct where Bosch writes parsed sensor values.
2. Contains fields for accel/gyro/other depending on call.

In your current code:

1. `bmi2_get_sensor_data(&bmi_sens_data, &bmi_dev)` updates it.
2. You print `bmi_sens_data.acc.x/y/z`.

Important beginner detail:

1. You do not manually fill this struct.
2. Bosch fills it after reading raw bytes from registers.

---

## 3) `bmi2_i2c_ctx_t` and `bmi2_i2c_ctx`

Ownership: [OURS]

What the type represents:

1. `hi2c` points to STM32 HAL I2C handle (`hi2c1`).
2. `dev_addr` stores BMI270 I2C address (7-bit form in your design).

What the instance does:

1. It is attached to Bosch via `bmi_dev.intf_ptr = &bmi2_i2c_ctx;`.
2. Bosch later passes this pointer back into read/write callbacks.

Why this pattern is important:

1. Bosch stays hardware-agnostic.
2. Your callback gets enough information to perform actual bus transfer.
3. The same callback pattern can support multiple sensors if each has its own context.

---

## 4) `last_imu_tick`

Ownership: [OURS]

What it does:

1. Stores last time IMU was polled.
2. Used in non-blocking scheduling pattern:
   - if `(HAL_GetTick() - last_imu_tick) >= 200`

Why this is good:

1. No `HAL_Delay` in main loop.
2. Other tasks can run between IMU reads.

---

## Common confusion to avoid

1. `bmi_dev` is not the raw sensor data.
2. `bmi_sens_data` is not configuration state.
3. `bmi2_i2c_ctx` is not Bosch internal object.
4. `last_imu_tick` is scheduler timing, not sensor timestamp.

---

## What you must master

1. Difference between driver object (`bmi_dev`) and data object (`bmi_sens_data`).
2. Why callbacks need context (`bmi2_i2c_ctx`).
3. Why non-blocking timer variable exists (`last_imu_tick`).

## What can be treated as black box

1. Bosch internal struct fields you do not touch directly.
2. Bosch parsing internals that fill `bmi_sens_data`.
