# 07 Startup Path Into IMU

## Code area

Inside `main()` before the while-loop.

Ownership split:

1. [GENERATED]/[DRIVER]: HAL and generated init functions.
2. [OURS]: startup ordering decisions and IMU bring-up policy.

---

## Why this section matters

Even if all IMU functions are correct, bad startup order can still break communication.

---

## Startup sequence used

1. `HAL_Init()`
2. `SystemClock_Config()`
3. `MX_GPIO_Init()`
4. `MX_USART2_UART_Init()`
5. `MX_I2C1_Init()`
6. `printf("Booting...")`
7. `bmi270_basic_init()`
8. set baseline ticks and LED initial state

---

## Why this order is correct

1. Clocks and HAL time base must exist before peripheral operations.
2. UART must be ready before debug printing.
3. I2C must be ready before sensor init.
4. Sensor init must happen before first sensor read.

---

## Failure policy in startup

If `bmi270_basic_init()` fails, code enters `Error_Handler()`.

Why good for learning:

1. Failure is loud and obvious.
2. You do not continue running with half-initialized hardware.

---

## Practical debugging signal

If serial shows `Booting...` but not `BMI270 ready...`:

1. UART path works,
2. failure is likely in IMU init phase.

This narrows your debug window immediately.

---

## What you must master

1. Dependency ordering in startup.
2. Why early-fail behavior is useful for bring-up.

## What can be black box

1. HAL internal startup internals.
