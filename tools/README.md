# IMU Live Plot + CSV Logger

This folder contains a small host-side tool to visualize UART IMU data and log it to CSV.

## 1) Firmware output format
The STM32 firmware should print one line per sample in this format:

`T,t_ms,ax,ay,az,gx,gy,gz`

Example:

`T,1234,-12,45,16320,0,0,0`

Note: gyro may be placeholder zeros until gyro is enabled in firmware.

## 2) Install dependencies (macOS)
```bash
python3 -m pip install pyserial matplotlib
```

## 3) Find your serial port
```bash
ls /dev/tty.usb*
```

## 4) Run tool
```bash
python3 tools/imu_live_plot_logger.py --port /dev/tty.usbmodemXXXX --baud 115200
```

Optional args:
- `--window 500` number of samples shown in plot window
- `--interval-ms 50` plot refresh period
- `--csv my_capture.csv` output filename

## 5) Output
- Live plot window with accel (top) and gyro (bottom)
- CSV file in current working directory with columns:
  `t_ms,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw`
