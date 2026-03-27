# 06 BMI270 Init bmi270_basic_init

## Function

`static int8_t bmi270_basic_init(void)`

Ownership: [OURS] orchestration function using [DRIVER] Bosch APIs.

---

## Purpose in one sentence

Prepare Bosch driver object, connect callbacks, configure accelerometer, enable it, and confirm sensor is ready.

---

## Why this function is important

This is your IMU bring-up script.

If this function is correct:

1. later sensor reads work.

If this function fails or order is wrong:

1. data reads fail or return invalid behavior.

---

## Step-by-step breakdown

### Step 1: local variables

1. `rslt` holds return codes from Bosch APIs.
2. `accel_cfg` holds accel settings.
3. `sens_list` identifies which sensor(s) to enable (here: accel only).

### Step 2: fill context object

1. `bmi2_i2c_ctx.hi2c = &hi2c1;`
2. `bmi2_i2c_ctx.dev_addr = BMI2_I2C_PRIM_ADDR;`

This is what callbacks need to reach real hardware.

### Step 3: fill `bmi_dev` callback/interface fields

1. set interface type to I2C
2. assign read callback
3. assign write callback
4. assign delay callback
5. assign context pointer
6. set read/write chunk length

This turns `bmi_dev` into a fully usable Bosch device object.

### Step 4: call `bmi270_init(&bmi_dev)`

What it does for you (high-level):

1. confirms communication,
2. prepares sensor and driver internal state,
3. reads chip-related baseline state.

You do not need to know its internals now; just call and check return code.

### Step 5: read current accel config

`bmi2_get_sensor_config(&accel_cfg, 1, &bmi_dev)`

Why this pattern is good:

1. start from known current config object,
2. then modify only fields you care about,
3. then write back.

### Step 6: set your chosen accel settings

1. ODR 100 Hz
2. range +-2g
3. bandwidth/averaging OSR2
4. performance mode

These are practical beginner settings for stable, readable bring-up.

### Step 7: apply config

`bmi2_set_sensor_config(&accel_cfg, 1, &bmi_dev)`

Now those values are written into sensor configuration registers.

### Step 8: enable accelerometer

`bmi2_sensor_enable(&sens_list, 1, &bmi_dev)`

Without this step, data path may stay inactive.

### Step 9: print chip id and return success

Useful for quick visual confirmation that init completed.

---

## Error handling pattern used

After each major Bosch call:

1. check `rslt`
2. print stage-specific error message
3. return early on failure

Why this is excellent for learning/debug:

1. You immediately know which stage failed.
2. You avoid hiding partial configuration states.

---

## Dependency order (must remember)

Correct order:

1. context
2. callback assignment
3. `bmi270_init`
4. get config
5. modify fields
6. set config
7. enable sensor

Changing order can break bring-up.

---

## What you must master

1. Why callback/context assignment comes before any Bosch API use.
2. Why return-code checks are mandatory in embedded.
3. Why enable step is separate from config step.

## What can be black box

1. Bosch internals inside `bmi270_init`.
2. Bosch internal register details for each config field.
