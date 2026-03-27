# 04 State Machines

## What a state machine is

A state machine is a small set of modes plus rules for moving between modes when events happen.

## Your current LED state machine

States:

1. Mode 0: LED off
2. Mode 1: LED on
3. Mode 2: LED blink

Transition event:

1. Debounced button press event

Transition rule:

1. `led_mode = (led_mode + 1) % 3`

## Why this is useful

1. Behavior is predictable.
2. Easy to explain and debug.
3. Easy to add more modes later.

## Real-world analogy

It is like a washing machine mode dial: each button press rotates to the next fixed mode.
