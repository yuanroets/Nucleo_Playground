# 01 Firmware Flow Big Picture

## Mental model

1. Hardware event happens (button press, UART byte, timer event).
2. Hardware sets an interrupt flag.
3. NVIC routes execution to the correct IRQ handler.
4. IRQ handler runs quickly.
5. IRQ path signals main loop using a flag or counter.
6. Main loop performs actual behavior.

## Real-world analogy

A doorbell rings:

1. You do not cook dinner at the door.
2. You note that someone arrived.
3. You return to normal flow and then handle it safely.

In firmware terms: interrupt sets a small event marker, main loop does the real work.

## Why this pattern matters

1. Better responsiveness.
2. Cleaner design.
3. Easier debugging.
4. Scales when project complexity grows.
