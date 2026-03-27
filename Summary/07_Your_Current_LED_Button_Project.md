# 07 Your Current LED Button Project

## Code files

1. `Meester_Playground/Core/Src/main.c`
2. `Meester_Playground/Core/Src/stm32l4xx_it.c`

## Startup behavior

1. Board initializes HAL and peripherals.
2. LED is forced off.
3. EXTI IRQ is enabled for button line group.
4. Main loop waits for events.

## Button press sequence

Press 1:

1. ISR path raises software event.
2. Main loop consumes event.
3. Mode 0 to 1.
4. LED turns on.

Press 2:

1. Event raised and consumed again.
2. Mode 1 to 2.
3. Blink reference reset.
4. LED starts toggling periodically.

Press 3:

1. Event raised and consumed again.
2. Mode 2 to 0.
3. LED forced off.

Then cycle repeats.

## Design quality check

1. ISR does minimal work.
2. Main loop handles behavior.
3. Timing is non-blocking.
4. Debounce is present.
