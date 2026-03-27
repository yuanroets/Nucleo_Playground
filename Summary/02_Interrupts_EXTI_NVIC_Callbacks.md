# 02 Interrupts EXTI NVIC Callbacks

## Key components

1. EXTI: hardware block that detects selected GPIO edge events.
2. NVIC: interrupt controller that decides which IRQ handler runs.
3. IRQ handler: C function entered by CPU for a given interrupt vector.
4. HAL handler: helper that processes EXTI pending state.
5. Callback: user hook where you put project-specific logic.

## Exact flow used in this project

1. Button edge occurs on pin line 13.
2. EXTI marks line 13 as pending.
3. NVIC invokes `EXTI15_10_IRQHandler`.
4. Handler calls `HAL_GPIO_EXTI_IRQHandler(Button_Pin)`.
5. HAL checks and clears pending state.
6. HAL calls `HAL_GPIO_EXTI_Callback(GPIO_Pin)`.
7. Your callback sets a software event flag.
8. Main loop reacts to that event.

## Why callback name must match exactly

`HAL_GPIO_EXTI_Callback` is a weak HAL hook. You override it by using the exact same function name and signature. If the name changes, HAL will call its own default weak version instead of your logic.

## EXTI line groups

1. EXTI0 handles line 0.
2. EXTI1 handles line 1.
3. EXTI2 handles line 2.
4. EXTI3 handles line 3.
5. EXTI4 handles line 4.
6. EXTI9_5 handles lines 5 to 9.
7. EXTI15_10 handles lines 10 to 15.

## Multiple EXTI pins

Use one callback and branch by `GPIO_Pin` value. You still must enable the correct IRQ groups in NVIC.
