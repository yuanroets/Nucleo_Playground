# 04 Bosch Callback bmi2_i2c_write

## Function

`static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)`

Ownership: [OURS] callback function, called by [DRIVER] Bosch code.

---

## What this function does

This is the write twin of `bmi2_i2c_read`.

It writes bytes from `reg_data` into BMI270 registers starting at `reg_addr`.

---

## Inputs, decoded

1. `reg_addr`: first register address to write.
2. `reg_data`: source buffer (bytes Bosch wants to send).
3. `len`: number of bytes to write.
4. `intf_ptr`: your context pointer (I2C handle + sensor address).

---

## Context pointer role

Same pattern as read callback:

1. Cast `void *intf_ptr` to `bmi2_i2c_ctx_t *`.
2. Use `ctx->hi2c` and `ctx->dev_addr` in HAL call.

No context pointer means callback would not know:

1. which I2C peripheral,
2. which target address.

---

## Why pointer checks are still necessary

Even though your code sets context correctly, callback API is generic and should defend itself.

Checks protect against:

1. null context
2. null I2C handle
3. null data pointer
4. zero-length write

This avoids undefined behavior and hard faults.

---

## HAL write call behavior

`HAL_I2C_Mem_Write(...)` handles the register write protocol.

Conceptually:

1. START
2. Sensor address + write
3. Register address
4. Write N bytes from `reg_data`
5. STOP

Your function does not manually toggle pins; HAL does that.

---

## Why `(ctx->dev_addr << 1)` again

Same reason as read callback:

1. Stored address is 7-bit logical format.
2. HAL memory API expects shifted transfer address convention.

---

## Return code mapping

1. HAL success -> Bosch success return.
2. HAL failure -> Bosch failure return.

Bosch uses this to decide if configuration write succeeded.

---

## Where writes happen in your current flow

During `bmi270_basic_init`:

1. `bmi2_set_sensor_config(...)` uses write callback internally.
2. `bmi2_sensor_enable(...)` also writes control registers.

So this callback is critical even before first sensor data read.

---

## What you must master

1. Read and write callbacks are symmetric bridge functions.
2. Same context pointer pattern applies in both.
3. Address shift convention must be consistent.

## What can be black box

1. HAL write internals.
2. Bosch internal register map handling.
