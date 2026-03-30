# Embedded Summary Index

This folder is your personal embedded programming reference for STM32 projects.

## How to use this folder

1. Start with `01_Firmware_Flow_Big_Picture.md` when you want the big picture.
2. Use `02_Interrupts_EXTI_NVIC_Callbacks.md` when working with button and IRQ logic.
3. Use `07_Your_Current_LED_Button_Project.md` to map theory to your current code.
4. Use `08_Power_Safety_Nucleo_L476RG.md` before changing power setup.
5. Use `10_IMU_BMI270_I2C_Cheat_Sheet.md` for IMU/I2C/Bosch SensorAPI bring-up and troubleshooting.
6. Use `IMU/README.md` for deep function-by-function IMU code explanations.
7. Use `12_BMP390_I2C2_Cheat_Sheet.md` for barometer pressure/temperature bring-up on `hi2c2`.
8. Add small lessons learned after each session.

## Current project references

- `Meester_Playground/Core/Src/main.c`
- `Meester_Playground/Core/Src/stm32l4xx_it.c`

## Core rule for clean embedded code

Keep ISR code short, keep behavior logic in the main loop, and use state machines for predictable behavior.
