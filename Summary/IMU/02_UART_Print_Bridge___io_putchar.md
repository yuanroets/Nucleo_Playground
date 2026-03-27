# 02 UART Print Bridge __io_putchar

## Function

`int __io_putchar(int ch)`

Ownership: [OURS], using [DRIVER] HAL UART API.

---

## Why this function exists

`printf` is generic C text output. On MCU there is no default console unless you map it.

This function maps each printed character to USART2 transmit.

Without it:

1. `printf("Booting")` may compile,
2. but nothing appears in your serial monitor.

---

## Step-by-step behavior

1. Input `ch` arrives from stdio layer.
2. Cast to `uint8_t` because UART sends bytes.
3. Call `HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY)`.
4. Return original `ch` as stdio expects.

---

## Parameter meaning in HAL call

`HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY)`

1. `&huart2`: which UART peripheral to use.
2. `&c`: pointer to byte buffer.
3. `1`: send one byte.
4. `HAL_MAX_DELAY`: block until done.

Why one byte:

1. `__io_putchar` runs per-character.
2. `printf` repeats this function for the whole string.

---

## Blocking behavior: is this okay?

Yes for now. You print low-rate debug text.

Tradeoff:

1. Simple and reliable.
2. Slower if you print very frequently.

For beginner bring-up this is a good choice.

---

## Common mistakes

1. Forgetting to initialize UART before first printf.
2. Wrong baud rate in terminal.
3. Missing retarget function or wrong function name for toolchain.

---

## What you must master

1. Why retarget is required for printf on MCU.
2. How HAL function parameters map to hardware behavior.

## What can be black box

1. HAL UART internal driver implementation.
2. libc printf internals.
