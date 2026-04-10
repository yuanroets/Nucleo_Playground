# 07 Your Current LED Button Project

## Code files

1. `Meester_Playground/Core/Src/main.c`
2. `Meester_Playground/Core/Src/stm32l4xx_it.c`

## Startup behavior

1. Board initializes HAL and peripherals.
2. LED is forced off.
3. EXTI IRQ is enabled for button line group.
4. Main loop waits for button events and periodic sensor tasks.

## Button press sequence (current)

Press 1 (start logging):

1. ISR path raises software event.
2. Main loop consumes event.
3. Firmware mounts SD card and opens three files: `IMU_DATA.CSV`, `BARO_DATA.CSV`, and `GPS_DATA.CSV`.
4. If mount returns no filesystem, firmware reports it and exits start path immediately.
5. Headers are written to all files and logging starts only when all opens/writes succeed.
6. LED enters blink state while logging is active.

Press 2 (stop logging):

1. Event raised and consumed again.
2. All three files are synced and closed.
3. Logging state is cleared.
4. LED is forced off.

## Design quality check

1. ISR does minimal work.
2. Main loop handles behavior.
3. Timing is non-blocking.
4. Debounce is present.
5. Unformatted SD cards fail fast without blocking the firmware.

## Runtime robustness notes (April 2026)

1. SD `f_sync()` is now centralized in one main-loop task and runs every 2 seconds.
2. Per-sample logging functions no longer call `f_sync()`, which avoids stacked sync stalls.
3. I2C callback transfers for BMI270 and BMP390 now use finite timeouts (10 ms), not `HAL_MAX_DELAY`.
4. GPS PPS flag handling runs in the main loop, so PPS updates are consumed continuously after startup.
5. Duplicate EXTI15_10 priority override was removed; PPS now keeps the configured NVIC priority.
6. FATFS `USER_diskio` now has a real SPI2 SD implementation (init/read/write/ioctl) instead of stubs that always returned `STA_NOINIT`.
7. `FR_NO_FILESYSTEM` on button press can now be treated as a real card/filesystem/wiring issue, not a missing disk backend.

## Current CSV fields

1. `IMU_DATA.CSV` logs `Sys_Time`, `UTC_GPS`, `AccX_g`, `AccY_g`, `AccZ_g`, `GyroX_dps`, `GyroY_dps`, `GyroZ_dps`.
2. `BARO_DATA.CSV` logs `Sys_Time`, `UTC_GPS`, `Temp_C`, `Press_kPa`.
3. `GPS_DATA.CSV` logs `Sys_Time`, `UTC_GPS`, `Lat_deg`, `Lon_deg`, `Alt_m`, `Sats`.
4. IMU and baro tasks log at their own sensor rates, while GPS file logging runs at 1 Hz.
5. If GPS has no valid fix, UTC/Lat/Lon/Alt fields are kept empty while IMU and baro logging continues.
