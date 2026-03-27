# 05 Debounce

## Why debounce exists

Mechanical buttons bounce electrically. One physical press can produce multiple rapid edge transitions.

Without debounce, one press may look like many presses.

## Current debounce logic

1. Save timestamp of last accepted press.
2. On interrupt, read current time.
3. Accept press only if enough time passed.

Current threshold in your code:

1. 150 ms

## Tradeoff

1. Lower threshold feels faster but may miscount.
2. Higher threshold is stable but may ignore very fast repeated presses.

## Startup note

If `last_button_tick` starts at 0, a very early press right after boot can be filtered. Usually this is acceptable.
