# Meester_Playground

STM32CubeMX-generated Nucleo-L476RG workspace with a small amount of hand-written application code, sensor bring-up, and supporting documentation.

## Where to work

- `Core/Src/main.c` is the main application entry point.
- `Core/Src/stm32l4xx_it.c` contains interrupt handlers and should stay small.
- `Summary/` is the project knowledge base and should be updated when the firmware behavior changes.
- `tools/` contains host-side Python helpers for RTT logging and plotting.
- `third_party/` contains vendor sensor APIs and should usually be left alone.

## Edit rules

- Prefer changes inside `USER CODE` blocks in generated STM32 files.
- Keep interrupts short and move behavior into the main loop or small state machines.
- Prefer non-blocking timing with `HAL_GetTick()` for periodic work.
