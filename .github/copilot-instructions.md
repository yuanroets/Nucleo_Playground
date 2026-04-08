# Copilot Instructions for Meester_Playground

This is an STM32CubeMX-generated Nucleo-L476RG project. Keep the generated structure intact and work mainly inside the `USER CODE` sections in `Core/Src/*.c` and `Core/Inc/*.h`.

Project rules:
- Preserve CubeMX-generated init code, linker/startup files, and vendor libraries unless the task explicitly requires changing them.
- Treat `Core/Src/main.c` as the main application entry point and keep blocking work out of interrupts.
- Keep ISRs short; put behavior in the main loop or small state machines.
- Prefer non-blocking timing with `HAL_GetTick()` and simple tick-based scheduling.
- Treat Bosch SensorAPI code as driver-layer integration; use the provided callback/context pattern instead of rewriting vendor code.
- Use `Summary/` as the project knowledge base for explanations, lessons learned, and wiring diagrams.
- Use `tools/` for host-side Python helpers such as RTT logging and plotting.
- Do not spend effort on `build/`, `Debug/`, `.settings/`, or other generated artifacts unless the task is specifically about build/debug setup.
- Prefer clear, compact code and ASCII-only edits unless the file already uses other characters.
- When adding documentation, update the Summary index rather than scattering notes across the repo.
