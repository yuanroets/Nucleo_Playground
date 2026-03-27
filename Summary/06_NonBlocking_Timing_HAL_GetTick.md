# 06 Non-Blocking Timing with HAL GetTick

## What HAL_GetTick returns

Milliseconds elapsed since system start.

## Why non-blocking timing

1. Keeps main loop responsive.
2. Allows handling events while timing periodic behavior.
3. Avoids blocking delays in core logic.

## Pattern

1. Save reference time.
2. Read current time each loop.
3. Compute elapsed time.
4. If elapsed >= interval, do action and refresh reference.

## Your blink logic

Reference: `last_blink_tick`

Current time: `now = HAL_GetTick()`

Condition: `(now - last_blink_tick) >= 200`

Action: toggle LED and set `last_blink_tick = now`

## Important detail

Toggle every 200 ms means LED changes state every 200 ms. A full on-off cycle takes about 400 ms.
