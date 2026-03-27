# 10 IMU + BMI270 + I2C Cheat Sheet

## Why this document exists

This chapter is your quick-reference for:

1. What an IMU is and what BMI270 gives you.
2. How I2C works at the electrical and protocol level.
3. How Bosch SensorAPI is structured and why we use it.
4. Exactly what is already integrated in your STM32L476 project.
5. What to check first when something does not work.

---

## 1) IMU fundamentals in one page

An IMU (Inertial Measurement Unit) measures motion-related quantities.

For your current sensor (BMI270), the main blocks are:

1. Accelerometer (linear acceleration on X/Y/Z).
2. Gyroscope (angular velocity on X/Y/Z).
3. Internal digital logic/firmware for filtering, timing, and feature functions.

### Accelerometer intuition

Accelerometer outputs include gravity. If the board is still on a desk:

1. One axis should read near +1g or -1g (depending on orientation).
2. The other two axes should read near 0g.

So a static accelerometer is already useful to estimate tilt/orientation relative to gravity.

### Gyroscope intuition

Gyroscope reads rotational speed:

1. Near 0 when still.
2. Positive/negative values while rotating.

Gyro is great for short-term rotation tracking but drifts if integrated too long without correction.

---

## 2) BMI270: what it is and why it is good for your project

BMI270 is a low-power 6-axis IMU from Bosch.

Why it fits your path:

1. Industry-standard embedded sensor with robust documentation.
2. Works over I2C or SPI, so transport can change later.
3. Bosch SensorAPI is plain C and portable to custom PCB firmware.
4. You can start simple (polling accelerometer) and scale to interrupts/FIFO/features later.

### Typical data path

1. MCU initializes BMI270.
2. MCU configures accel/gyro ODR/range/filter.
3. MCU enables chosen sensors.
4. MCU periodically reads sensor registers (or FIFO/interrupt-driven path later).
5. Raw data is converted/scaled by application code if needed.

---

## 3) I2C cheat sheet (practical embedded view)

## 3.1 Bus wiring model

I2C uses two shared lines:

1. SCL (clock).
2. SDA (data).

Both are open-drain/open-collector style:

1. Devices only pull lines low.
2. Pull-up resistors bring lines high.

That means:

1. If pull-ups are missing/too weak/too strong, communication can fail.
2. Multiple devices can share the bus using unique addresses.

## 3.2 Addressing details that usually confuse beginners

Sensor datasheets often show 7-bit addresses (example BMI270: 0x68 primary, 0x69 alternate).

HAL I2C memory functions on STM32 typically expect the left-shifted value in the transfer call. In your glue code that is done with:

1. dev_addr is stored as 7-bit value (for readability).
2. Call passes dev_addr << 1 to HAL.

This is one of the most common causes of I2C bring-up bugs.

## 3.3 Transaction concept

For register-style sensors, a read usually looks like:

1. START
2. Send slave address + write bit
3. Send register address
4. RESTART
5. Send slave address + read bit
6. Read N data bytes
7. STOP

Your HAL memory read/write APIs implement this pattern.

## 3.4 Electrical sanity checks

If the bus does not work, verify:

1. Common GND between Nucleo and sensor board.
2. Correct voltage domain compatibility (SparkFun BMI270 breakout supports 3.3V systems).
3. SDA/SCL not swapped.
4. Pull-ups present once (not accidentally over-loaded by too many strong pull-ups).
5. Correct sensor address selection jumper state.

---

## 4) SparkFun Qwiic BMI270 board notes

You are using the SparkFun breakout mainly as hardware:

1. Sensor + level/power support + routing convenience.
2. Qwiic connector optional (you can use breadboard headers).

Software strategy chosen:

1. Use Bosch SensorAPI as primary firmware driver.
2. Use SparkFun docs/schematic/jumper info for hardware setup.

Why this is the right architecture for your future custom PCB:

1. Bosch API maps directly to the real chip behavior.
2. MCU-side transport wrappers are yours and portable.
3. You are not locked into Arduino-specific abstractions.

---

## 5) Bosch SensorAPI summary (the important mental model)

Think of Bosch API as two layers:

1. Generic sensor logic in Bosch library.
2. Platform-specific transport/delay callbacks provided by you.

## 5.1 Core object

The central context is struct bmi2_dev.

It stores:

1. Interface type (I2C/SPI).
2. Function pointers for read/write/delay.
3. Interface pointer (your context with HAL handle/address).
4. Internal driver state such as chip id and config data.

## 5.2 Required callbacks you implemented

1. read(reg, buffer, len, intf_ptr)
2. write(reg, buffer, len, intf_ptr)
3. delay_us(period, intf_ptr)

These callbacks are the bridge between Bosch code and STM32 HAL.

## 5.3 Initialization flow you should remember

Minimal flow for bring-up:

1. Fill bmi2_dev callbacks + interface fields.
2. Call bmi270_init().
3. Get current sensor config (bmi2_get_sensor_config).
4. Modify needed fields (ODR, range, filter, perf mode).
5. Set config (bmi2_set_sensor_config).
6. Enable desired sensors (bmi2_sensor_enable).
7. Poll with bmi2_get_sensor_data.

If this sequence is wrong/out-of-order, initialization or reads may fail.

---

## 6) What is already implemented in your project

## 6.1 Build system integration

In CMake, Bosch sources are compiled directly into your firmware:

1. third_party/BMI270_SensorAPI/bmi2.c
2. third_party/BMI270_SensorAPI/bmi270.c

And include path is added for Bosch headers.

## 6.2 CubeMX/HAL peripheral setup relevant to IMU

1. I2C1 is enabled and initialized in startup path.
2. GPIO mapping in MSP:
   - PB6 -> I2C1_SCL
   - PB7 -> I2C1_SDA
3. I2C addressing mode is 7-bit.
4. Analog filter enabled, digital filter set to 0.

## 6.3 Application-level IMU integration in main.c

Implemented pieces:

1. UART printf bridge (so sensor debug lines go to USART2 terminal).
2. I2C context struct holding HAL I2C handle + device address.
3. Bosch read/write wrappers using HAL_I2C_Mem_Read / HAL_I2C_Mem_Write.
4. Microsecond delay callback (busy-wait based).
5. bmi270_basic_init() routine for init + accel config + accel enable.
6. Main loop polling every 200 ms with raw accel print.

## 6.4 Current accel config values

Configured in code as:

1. ODR: 100 Hz
2. Range: +-2g
3. Bandwidth/averaging: OSR2/AVG2
4. Filter performance: performance optimized mode

## 6.5 Current behavior at runtime

On successful boot:

1. Prints boot line.
2. Initializes BMI270.
3. Prints BMI270 chip id.
4. Every 200 ms prints raw accel x/y/z values.

If init/read fails, error code is printed and init path falls to Error_Handler during startup.

---

## 7) Detailed code walkthrough (what each part does)

This section is a practical tour of your current implementation, from build to runtime.

## 7.1 Build system: how Bosch code gets into the firmware

Your CMake setup adds Bosch driver source files directly into the same firmware image as your STM32 application code.

That means:

1. The linker can resolve calls like bmi270_init() and bmi2_get_sensor_data().
2. You do not need a separate static library step.
3. Vendor driver and your app code compile under the same build options.

## 7.2 Global objects: what they represent in memory

At file scope in main.c, each object has a specific role:

1. hi2c1 is the HAL handle for I2C peripheral state (register base, init config, transfer state).
2. huart2 is the HAL handle used by printf bridge for serial output.
3. bmi_dev is Bosch driver context (interface type, callbacks, internal state).
4. bmi_sens_data is the latest sensor sample container.
5. bmi2_i2c_ctx stores your platform context: which HAL I2C handle to use and which sensor address.
6. last_imu_tick timestamps the last IMU poll for non-blocking periodic reads.

## 7.3 printf bridge: why terminal logs work

The __io_putchar() function routes each character from printf to HAL_UART_Transmit on USART2.

Why this matters:

1. Your startup and error diagnostics are visible immediately.
2. You can validate sensor behavior without debugger breakpoints.
3. It creates a low-friction bring-up loop: flash -> observe logs -> adjust.

## 7.4 Bosch callback adapters: the most important integration layer

Bosch SensorAPI is platform-independent. It cannot call STM32 HAL directly, so it calls function pointers you provide.

Your wrappers do exactly that:

1. bmi2_i2c_read() maps Bosch read requests to HAL_I2C_Mem_Read.
2. bmi2_i2c_write() maps Bosch write requests to HAL_I2C_Mem_Write.
3. bmi2_delay_us() gives Bosch required timing delays.

Important detail in read/write wrappers:

1. Sensor address is stored as 7-bit value in context.
2. HAL transfer call uses shifted address (dev_addr << 1).

That address handling is a common source of bugs, and your implementation handles it correctly.

## 7.5 bmi270_basic_init(): exact startup sequence

The init function performs a strict sequence:

1. Fill I2C context with hi2c1 and BMI2_I2C_PRIM_ADDR.
2. Fill bmi_dev interface fields and callback pointers.
3. Call bmi270_init() to load/initialize BMI270 driver state and chip communication.
4. Read current accel config with bmi2_get_sensor_config().
5. Modify accel parameters (ODR/range/filter/perf).
6. Write config back with bmi2_set_sensor_config().
7. Enable accel using bmi2_sensor_enable().
8. Print chip id when successful.

Why this is robust:

1. Every step checks return code.
2. Failures print an explicit stage message (init/get/set/enable).
3. Function exits early on failure, avoiding hidden partial state.

## 7.6 main() startup flow: order and reasons

Startup in main() follows dependency order:

1. HAL_Init() and SystemClock_Config() prepare MCU runtime foundation.
2. MX_GPIO_Init(), MX_USART2_UART_Init(), MX_I2C1_Init() bring up required peripherals.
3. Boot log is printed.
4. bmi270_basic_init() runs only after I2C exists.
5. If IMU init fails, code enters Error_Handler for safe-stop behavior.
6. Tick baselines are initialized for LED and IMU periodic tasks.

This order prevents using peripherals before they are initialized.

## 7.7 main loop execution model: cooperative tasks

Your while(1) loop runs three independent behaviors cooperatively:

1. Button event handling updates LED mode state.
2. LED blink task toggles at 200 ms when blink mode is active.
3. IMU task runs every 200 ms and requests fresh sensor data.

This is non-blocking timing based on HAL_GetTick() differences, so tasks share CPU time predictably.

## 7.8 IMU polling/read path in steady state

Every 200 ms:

1. Compare HAL_GetTick() against last_imu_tick.
2. Call bmi2_get_sensor_data(&bmi_sens_data, &bmi_dev).
3. On success print raw accel x/y/z.
4. On failure print Bosch return code.

This is intentionally simple and ideal for bring-up before moving to interrupt/FIFO designs.

## 7.9 CubeMX-generated I2C setup used by the IMU

Your generated I2C configuration gives the hardware base layer for the callbacks:

1. I2C1 configured in 7-bit mode.
2. CubeMX timing value is applied via hi2c1.Init.Timing.
3. Analog filter enabled; digital filter disabled.
4. MSP binds pins PB6/PB7 to I2C1 SCL/SDA in AF4 open-drain with pull-ups.

This matches the expected electrical/protocol setup for BMI270 I2C use.

## 7.10 End-to-end mental model in one line

Main loop asks Bosch for data, Bosch calls your callbacks, callbacks call STM32 HAL I2C, bytes come back from BMI270, and main loop prints them on UART.

---

## 8) Data interpretation quick reference

Current output is raw counts, not yet converted to m/s^2.

For first validation:

1. Rotate board by 90 degrees and watch which axis takes gravity.
2. Sign should flip when orientation flips.
3. Magnitude trend should stay consistent for gravity axis when static.

Later, you can add conversion/scaling once raw behavior is trusted.

---

## 9) Polling vs interrupt for IMU in your stage

Current design uses polling (every 200 ms), which is correct for first bring-up.

Why polling first:

1. Fewer moving parts to debug.
2. Easy serial visibility.
3. Lets you confirm electrical + address + config path quickly.

Later upgrade path:

1. Enable data-ready interrupt pin.
2. Configure interrupt mapping in BMI270.
3. In ISR set a flag only.
4. Read data in main loop (non-blocking pattern).

---

## 10) Common failure patterns and fastest checks

## 10.1 Init fails immediately

Check:

1. Wiring and power first.
2. Correct 7-bit address selection (0x68 vs 0x69).
3. Shift-left handling into HAL call.
4. I2C peripheral actually initialized before bmi270_basic_init().

## 10.2 Reads always zero or nonsense

Check:

1. Sensor enable call result.
2. Config set/get return codes.
3. Not reading too early before init sequence completes.
4. Shared bus conflicts or incorrect pull-up situation.

## 10.3 Intermittent errors

Check:

1. Cable/breadboard contact quality.
2. Clock speed/pull-up balance.
3. Overly long wiring.
4. Timing edge cases in delay callback.

---

## 11) CubeMX regeneration safety rule

Keep custom IMU logic only in USER CODE sections of generated files.

Safe custom zones currently used:

1. Includes in USER CODE include section.
2. Custom globals and helper function prototypes.
3. Callback/helper function definitions in USER CODE blocks.
4. Runtime init and polling logic in USER CODE blocks in main().

This preserves your changes when regenerating from .ioc.

---

## 12) File map for this IMU integration

Project files involved:

1. Core/Src/main.c (BMI270 glue code + polling loop)
2. Core/Src/stm32l4xx_hal_msp.c (I2C1 pin/peripheral MSP setup)
3. CMakeLists.txt (Bosch sources/includes)
4. third_party/BMI270_SensorAPI/* (vendor driver source)

---

## 13) Next practical steps

When you continue IMU work, use this sequence:

1. Confirm stable raw accel readings while rotating board by hand.
2. Add a tiny I2C scanner utility (optional but useful for bus diagnostics).
3. Add scaled accel output (g or m/s^2) with clear units.
4. Enable gyro and print accel+gyro together.
5. Move toward non-blocking architecture (state machine or event flags) if data flow grows.

---

## 14) One-paragraph memory hook

Bosch owns sensor intelligence, your STM32 code owns transport. The bmi2_dev struct connects the two through read/write/delay callbacks. I2C is two open-drain wires with pull-ups and 7-bit addresses, and HAL calls usually want shifted address values. Your project already compiles with Bosch BMI270 driver, initializes IMU over I2C1, enables accelerometer at 100 Hz and 2g range, and prints raw XYZ acceleration periodically over UART for bring-up and validation.
