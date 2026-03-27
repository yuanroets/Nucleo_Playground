# IMU Deep-Dive Function Notes

This folder is a function-by-function study guide for the IMU integration in `Core/Src/main.c`.

Use this order when learning:

1. `01_IMU_Global_State_And_Context.md`
2. `02_UART_Print_Bridge___io_putchar.md`
3. `03_Bosch_Callback_bmi2_i2c_read.md`
4. `04_Bosch_Callback_bmi2_i2c_write.md`
5. `05_Bosch_Callback_bmi2_delay_us.md`
6. `06_BMI270_Init_bmi270_basic_init.md`
7. `07_Startup_Path_Into_IMU.md`
8. `08_Main_Loop_IMU_Polling_Task.md`
9. `09_I2C_Peripheral_Init_MX_I2C1_Init.md`

## How to use these notes

1. Open one file at a time.
2. Keep `Core/Src/main.c` open side by side.
3. Read section "What you must master" first.
4. Read section "What can be used as a black box" second.
5. Return to firmware and trace execution in order.

## Ownership reminder

- [OURS]: You should deeply understand this logic.
- [DRIVER]: You should know what API does and parameters to pass.
- [GENERATED]: Understand settings/meaning, not boilerplate internals.
