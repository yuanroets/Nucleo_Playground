---
description: "Guidance for STM32 application code in Core/Src and Core/Inc"
applyTo: "Core/**/*.{c,h}"
---

Follow the STM32CubeMX pattern already used in this project.

- Keep user changes inside `USER CODE BEGIN/END` blocks when possible.
- Do not rewrite generated peripheral init functions unless the task explicitly asks for it.
- Keep interrupt handlers minimal and defer work to the main loop or state flags.
- Prefer explicit context structs for callbacks and peripheral-dependent code.
- Use non-blocking scheduling with tick counters rather than busy waits where practical.
- Keep comments short and technical; avoid duplicating what the code already says.
- If a change affects application behavior, update the relevant notes under `Summary/`.
