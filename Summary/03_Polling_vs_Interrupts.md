# 03 Polling vs Interrupts

## Polling

1. Main loop repeatedly reads pin state.
2. Easy to start with.
3. Can miss short events if loop is slow or blocked.
4. Long delays make it worse.

## Interrupts

1. Hardware notifies CPU when event happens.
2. Better response for short events.
3. More scalable as features grow.
4. Requires disciplined ISR design.

## Practical recommendation

1. Use interrupt for event detection.
2. Keep ISR tiny.
3. Process behavior in main loop.

This gives both responsiveness and maintainability.
