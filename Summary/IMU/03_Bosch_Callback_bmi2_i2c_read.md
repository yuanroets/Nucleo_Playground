# 03 Bosch Callback bmi2_i2c_read

## Function

`static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)`

Ownership: [OURS] callback function, called by [DRIVER] Bosch code.

This is the most important bridge function in your IMU integration.

---

## First correction of terminology

`BMI2_INTF_RETURN_TYPE` is a type, not a function.

In Bosch headers it is defined as an integer type used for callback status codes.

So this line means:

1. Function name is `bmi2_i2c_read`.
2. Return type is `BMI2_INTF_RETURN_TYPE`.

---

## Why Bosch needs this callback

Bosch SensorAPI does not know STM32 HAL.

So Bosch asks you to provide function pointers for:

1. read
2. write
3. delay

When Bosch wants register bytes, it calls your read callback through `bmi_dev.read(...)`.

---

## Inputs, decoded carefully

1. `reg_addr`: first BMI register address to read.
2. `reg_data`: destination buffer pointer provided by Bosch.
3. `len`: number of bytes Bosch requests.
4. `intf_ptr`: opaque pointer passed back to you (your context).

Important beginner point:

1. You do not allocate `reg_data` in this function.
2. You only fill it.

---

## Context pointer in plain language

`intf_ptr` is a generic `void *` so Bosch can pass any platform context.

In your code, you cast it to `bmi2_i2c_ctx_t *`.

That context struct gives you:

1. `hi2c` -> which STM32 I2C handle to use.
2. `dev_addr` -> which I2C device address to target.

So context pointer solves: "How does callback know which hardware to use?"

---

## Line-by-line logic flow

1. Cast `intf_ptr` to your context type.
2. Create `status` variable for HAL result.
3. Validate pointers and length.
4. Call HAL memory-read API.
5. Translate HAL status to Bosch return code.

---

## HAL read call and what it does

`HAL_I2C_Mem_Read(...)` performs register read transaction on I2C bus.

Conceptual bus sequence:

1. START
2. Sensor address + write
3. Register address
4. RESTART
5. Sensor address + read
6. Read N bytes
7. STOP

Those bytes land in `reg_data` buffer.

---

## Why address is shifted

You pass `(ctx->dev_addr << 1U)`.

Reason:

1. Your context stores normal 7-bit address.
2. This HAL API expects shifted address convention.

If this shift is wrong/missing, reads usually fail.

---

## Return value mapping

If HAL returns `HAL_OK`:

1. Return Bosch success constant.

Else:

1. Return failure value.

Bosch then decides whether to continue parse or report communication error.

---

## How this function gets called in your app

Call chain:

1. main loop calls `bmi2_get_sensor_data(...)`.
2. Bosch code calls internal register-read helper.
3. Bosch helper calls `dev->read(...)`.
4. `dev->read` points to your `bmi2_i2c_read`.
5. This function performs actual I2C read through HAL.

So even though you call one high-level Bosch API, this callback is the hardware path.

---

## Typical failure cases

1. `intf_ptr` not set to valid context.
2. `hi2c` not initialized yet.
3. Wrong address (0x68 vs 0x69 board jumper situation).
4. Missing shift for HAL address convention.
5. Wiring/pull-up issue on SCL/SDA.

---

## What you must master

1. Callback pattern with function pointer.
2. Context pointer cast and usage.
3. HAL read parameter meaning.
4. Return code translation.

## What can be black box

1. HAL internal state machine.
2. Bosch internal parser after read completes.
