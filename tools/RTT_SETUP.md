# SEGGER RTT Live IMU Plotting Setup

This guide shows how to stream IMU data from your STM32 over the ST-Link debugger using SEGGER RTT (no UART needed).

## What is RTT?

**SEGGER Real-Time Transfer (RTT)** allows your STM32 to send data to the host PC over the SWD debug connection at high speed with zero CPU overhead.

Benefits over UART:
- ✅ Uses existing ST-Link USB connection (no extra cable)
- ✅ No UART baud rate limitations → faster data stream
- ✅ Zero-copy ring buffer (no blocking I/O)
- ✅ Code runs at full speed (debug mode NOT required)

## Setup Steps

### 1. Build and Flash
```bash
# Build with new RTT files included
cd /Users/yuanroets/stm32/nucleo-l476rg-blink/Meester_Playground
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Flash to Nucleo (using STM32CubeIDE or STM32 VS Code extension)
```

### 2. Connect Debugger
- Keep ST-Link USB cable plugged into Nucleo board
- Code will run normally (full speed, NOT in debug mode)

### 3. Stream Data

Choose **ONE** of these options (in order of preference):

#### **Option A: Using ST-Link Probe (macOS/Linux) - Simplest**

Install `pyocd`:
```bash
pip install pyocd
```

Run capture:
```bash
cd tools
python3 rtt_capture.py
```

This streams IMU data to stdout. Pipe to plotter in another terminal:
```bash
# Terminal 1: Start RTT capture
python3 rtt_capture.py > /tmp/imu_stream &

# Terminal 2: Run plotter (create named pipe first)
mkfifo /tmp/imu_stream
cat /tmp/imu_stream | python3 imu_live_plot_logger_stdin.py
```

---

#### **Option B: Using ST Microelectronics Tools (Professional)**

If you have STM32CubeIDE installed, use its built-in System View tool:

1. In STM32CubeIDE: `Window → Show View → System Viewer`
2. Select your device and probe
3. Traces tab shows RTT output in real-time
4. Copy data and pipe to plotter

---

#### **Option C: Using JLinkExe (If you have J-Link probe)**

```bash
# Connect with JLinkExe
JLinkExe -Device STM32L476RG -If SWD -Speed 4000

# In JLinkExe console:
# rtt
# RTT0
# c
```

Then read RTT output in background and pipe to plotter.

---

#### **Option D: Manual GDB + RTT (Advanced)**

```bash
arm-none-eabi-gdb build/Debug/Meester_Playground.elf

(gdb) target remote localhost:3333          # OpenOCD or STLink Server
(gdb) load
(gdb) continue
```

Set breakpoint and read RTT buffer contents manually.

---

## What the Code Does

When you build and run:

1. `SEGGER_RTT_Init()` called in `main()` → sets up RTT buffer in RAM
2. `printf()` calls write to RTT buffer (via `__io_putchar()` hook)
3. RTT buffer is visible over SWD debug connection
4. Host tool reads it in real-time
5. Data flows: STM32 → SWD cable → Host → RTT Reader → Plotter

## Verification

If RTT is working, you should see in your RTT reader:
```
Booting...
BMI270 ready, chip id: 0x24
T,1234,512,100,-1050,45,-30,10
T,1240,520,105,-1045,48,-28,12
...
```

If not:
- ❌ Check ST-Link USB is plugged in
- ❌ Verify code compiled (check `third_party/SEGGER_RTT/` files exist)
- ❌ Ensure `SEGGER_RTT_Init()` is called before first `printf()`
- ❌ Try reflashing the board

## Troubleshooting

**"RTT buffer not found"**
- Make sure SEGGER_RTT files are compiled (check build log)
- Try a clean rebuild: `rm -rf build && cmake -B build`

**"No ST-Link detected"**
- Check USB connection: `ls /dev/tty*` should show a device
- Try: `pip install --upgrade pyocd`

**Data seems garbled**
- RTT buffer may be overflowing if sampling too fast
- Increase `BUFFER_SIZE_UP` in `SEGGER_RTT_Conf.h` (default 1 KB)

**Want to switch back to UART?**
- Just change `__io_putchar()` back to use `HAL_UART_Transmit()`
- Or run: `tools/imu_live_plot_logger.py --port /dev/ttyACM0`

---

## Files Modified

- `Core/Src/main.c` → RTT init + printf redirect
- `CMakeLists.txt` → includes SEGGER_RTT build
- `third_party/SEGGER_RTT/` → RTT library (created)

## Next Steps

Once RTT is working:
1. Run `python3 tools/rtt_capture.py` in one terminal
2. In another: `python3 tools/imu_live_plot_logger_stdin.py` (pipe from RTT capture)
3. See live 6-axis IMU plot (accel + gyro)

Questions? Check `Summary/` folder for IMU learning materials.
