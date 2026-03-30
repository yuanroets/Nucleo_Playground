# BMP390 I2C2 Cheat Sheet (Bosch SensorAPI)

This sheet mirrors the BMI270 pattern you already use, but for the Adafruit BMP390 barometer.

## 1) Build Wiring

- Driver cloned into: `third_party/BMP3_SensorAPI`
- Added source: `third_party/BMP3_SensorAPI/bmp3.c`
- Added include path: `third_party/BMP3_SensorAPI`

## 2) Runtime Wiring in `Core/Src/main.c`

- Include added: `#include "bmp3.h"`
- Separate Bosch device object: `struct bmp3_dev bmp_dev`
- Separate context struct for I2C2 + address: `bmp3_i2c_ctx_t`
- New callback bridge functions:
  - `bmp3_i2c_read(...)`
  - `bmp3_i2c_write(...)`
  - `bmp3_delay_us(...)`

## 3) Address Detect (Adafruit Friendly)

BMP390 boards can be wired as either:
- `0x77` (secondary)
- `0x76` (primary)

Firmware now probes both addresses on `hi2c2` by reading `BMP3_REG_CHIP_ID`.

## 4) Init Flow (`bmp390_basic_init`)

1. Detect address on `hi2c2`.
2. Fill Bosch callbacks and interface pointers.
3. `bmp3_init(&bmp_dev)`
4. Enable both pressure and temperature.
5. Set filter/ODR/oversampling.
6. Set `BMP3_MODE_NORMAL`.

Current config:
- ODR: `BMP3_ODR_25_HZ`
- Pressure OS: `BMP3_OVERSAMPLING_8X`
- Temperature OS: `BMP3_OVERSAMPLING_2X`
- IIR: `BMP3_IIR_FILTER_COEFF_3`

## 5) Main Loop Task

- Poll period: `BARO_VISUAL_REFRESH_MS = 40` ms (25 Hz)
- Read API: `bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp_data, &bmp_dev)`
- Output CSV line:
  - `P,t_ms,pressure_pa,temp_c_x100`

Example:
- `P,12345,100812,2487` means:
  - `t_ms=12345`
  - pressure `100812` Pa
  - temperature `24.87 C`

## 6) Health Counters

- `baro_read_ok_count`
- `baro_read_err_count`

Printed once per second together with IMU counters.

## 7) Quick Bring-Up Checks

- If init fails, check wiring and pullups on I2C2.
- Verify board power is 3.3V.
- Confirm RTT output shows: `BMP390 ready, addr:0x.. chip id:0x..`
- If reads fail intermittently, reduce cable length and verify ground continuity.

## 8) Next Step Ideas

- Add relative altitude estimate from pressure baseline.
- Add a low-pass or moving average for altitude trend.
- Merge IMU + BARO into one unified CSV schema when you are ready.
