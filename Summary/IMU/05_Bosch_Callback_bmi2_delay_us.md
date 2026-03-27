# 05 Bosch Callback bmi2_delay_us

## Function

`static void bmi2_delay_us(uint32_t period, void *intf_ptr)`

Ownership: [OURS] callback function used by [DRIVER] Bosch code.

---

## Why this function exists

Bosch driver needs timed gaps between certain operations.

Instead of implementing delay itself for every MCU, it calls your delay callback.

This keeps Bosch code portable.

---

## Inputs

1. `period`: delay length in microseconds.
2. `intf_ptr`: context pointer (unused in your current implementation).

Why pointer exists even when unused:

1. Common callback pattern across all platforms.
2. Some systems may need context for timer/peripheral access.

---

## How your current version works

1. Compute loop count from `SystemCoreClock` and requested microseconds.
2. Run for-loop that executes `__NOP()` each iteration.
3. Exit after rough equivalent of requested delay.

This is a busy-wait delay.

---

## Busy-wait means

1. CPU does nothing else while waiting.
2. Delay is approximate, not lab-grade precise.
3. Good enough for first-stage bring-up and driver timing requirements.

---

## Why `volatile` is used

`volatile uint32_t cycles;`

Prevents compiler from optimizing out the delay loop assumptions.

Without `volatile`, aggressive optimization might reduce or remove work.

---

## Why `__NOP()` is used

`__NOP()` is a no-operation assembly instruction.

Benefits:

1. Predictable tiny instruction.
2. Makes loop timing behavior more stable than an empty loop.

---

## Limitation to remember

This formula is intentionally simple. Real delay may vary with:

1. compiler optimization level,
2. instruction/cache effects,
3. clock changes.

For now, this simplicity is acceptable and useful for learning.

---

## What you must master

1. Why Bosch requires a delay callback.
2. Why blocking delay is acceptable at this stage.
3. Why `volatile` and `__NOP()` appear here.

## What can be black box

1. Exact microarchitectural cycle timing details.
