# Pointers, Callbacks, and SEGGER RTT - Learning Summary

**Date:** 26 March 2026  
**Context:** Setting up live IMU plotting over debug link (no UART)

---

## 📚 What We Built

A **real-time athlete monitoring system** that streams 6-axis IMU data (accel X,Y,Z + gyro X,Y,Z) at 200 Hz from STM32 to your laptop via ST-Link debugger, with live matplotlib visualization.

### Key Config:
- **Accelerometer:** 200 Hz, ±16g range (captures falls/impacts)
- **Gyroscope:** 200 Hz, ±2000 dps range (captures rotation)
- **Output:** SEGGER RTT over SWD cable (no UART needed!)
- **Format:** CSV → Python plotter

---

## 🎯 Complete Data Flow

```
BMI270 Sensor (I2C)
    ↓ (samples at 200 Hz)
STM32 Main Loop (every 5ms)
    ↓ (reads via I2C callbacks)
Format CSV: T,timestamp,ax,ay,az,gx,gy,gz
    ↓
SEGGER RTT Buffer (ring buffer in RAM)
    ↓ (over SWD cable)
ST-Link USB Debugger
    ↓
Your PC
    ↓ (rtt_capture.py)
Python Plotter
    ↓
Live 6-axis Graph
```

---

## 🔩 POINTER FUNDAMENTALS

### Basic Rules

```c
int i = 5;                    // Variable with value 5

&i                            // Address of i (where 5 lives)
                              // Example: 0x20000100

int *p = &i;                  // Pointer: stores the ADDRESS
                              // p holds: 0x20000100 (not 5!)

*p                            // Dereference: follow pointer, get value
                              // *p = 5 (same as i)

*p = 7;                       // Change value at address
                              // Now i = 7 also (same memory!)
```

### Memory Picture

```
MEMORY:
Address 0x20000100: [5]  ← i lives here

Pointer p:
  p holds: 0x20000100
  *p gives you: [5]
  
i and *p are the SAME location!
```

### Key Symbols

| Symbol | Meaning |
|--------|---------|
| `i` | The value itself |
| `&i` | Address OF i |
| `int *p` | p is a pointer (stores address) |
| `*p` | Dereference: get value from address |
| `p->field` | Dereference AND access field |

---

## 🎛️ THE OPAQUE POINTER PATTERN

### Problem We Solved

The **Bosch BMI270 driver is generic** — it can't know which **specific I2C bus** or **chip address** your project uses.

### Solution: Pass Your Config as Context

**Step 1: Define a context struct**
```c
typedef struct {
  I2C_HandleTypeDef *hi2c;    // Pointer to I2C handle. Handle to big to not use pointer
  uint16_t dev_addr;           // Chip address (0x68)
} bmi2_i2c_ctx_t;

bmi2_i2c_ctx_t bmi2_i2c_ctx;  // Create instance
```

**Step 2: Store your config**
```c
bmi2_i2c_ctx.hi2c = &hi2c1;       // Store ADDRESS of hi2c1
bmi2_i2c_ctx.dev_addr = 0x68;     // Store address value
```

**Step 3: Pass to Bosch as opaque pointer**
```c
bmi_dev.intf_ptr = &bmi2_i2c_ctx;  // Store ADDRESS of context
```

**Step 4: Bosch calls your callback with context**
```c
void bmi2_i2c_read(..., void *intf_ptr) {
  // intf_ptr holds: address of bmi2_i2c_ctx (0x20000200)
  
  // Cast it back to your type
  bmi2_i2c_ctx_t *ctx = (bmi2_i2c_ctx_t *) intf_ptr;
  
  // Now access your config
  ctx->hi2c;       // Get the I2C handle
  ctx->dev_addr;   // Get the address
}
```

### Memory Diagram

```
ADDRESSES:
0x20000100:  ┌─────────────────┐
             │  hi2c1 struct   │ (actual I2C handle)
             └─────────────────┘

0x20000200:  ┌─────────────────┐
             │ bmi2_i2c_ctx    │
             │  .hi2c → 0x20000100
             │  .dev_addr=0x68 │
             └─────────────────┘

0x20000300:  ┌─────────────────┐
             │ bmi_dev struct  │
             │  .intf_ptr → 0x20000200
             │  .read → function address
             └─────────────────┘

Flow:
  bmi_dev.intf_ptr stores: 0x20000200
  ↓
  Bosch passes: 0x20000200 to callback
  ↓
  We cast and access: ctx->hi2c → 0x20000100 → real handle
```

### Why This Pattern?

✅ **Bosch code 100% independent** of your hardware  
✅ **Multiple sensors** with different I2C buses  
✅ **Reusable** across projects  
✅ **Clean** callback design  

---

## 🔄 CASTING vs DEREFERENCING

### Critical Difference

```c
int i = 5;
int j;
int *p;

/* CASE A: Store address (same memory) */
p = &i;           // p points to i
*p = 7;           // Changes i to 7
                  // Both i and *p in same location

/* CASE B: Copy value (different memory) */
p = &j;           // p points to j
*p = i;           // Copy VALUE of i to j
*p = 7;           // Only j changes, i unchanged
                  // Different locations!
```

### In Your Code

```c
/* Stores ADDRESS: hi2c1 and ctx are linked */
bmi2_i2c_ctx.hi2c = &hi2c1;

/* Later, cast to get typed access */
bmi2_i2c_ctx_t *ctx = (bmi2_i2c_ctx_t *) intf_ptr;
ctx->hi2c;  // Same as hi2c1 (both point to same memory)
```

---

## 📡 SEGGER RTT vs UART

### UART (Old Way)

```
STM32 → UART TX pin → physical cable → PC UART
Bottleneck: 115,200 baud = 11.5 KB/sec max
At 200 Hz × 50 bytes = 10 KB/sec (barely fits!)
```

### SEGGER RTT (New Way)

```
STM32 → SWD cable → ST-Link → USB → PC
Speed: 400+ KB/sec (no bottleneck!)
Already plugged in for flashing
Uses SWD debug connection (no extra cable)
```

### Why RTT?

✅ **Faster** (debug link not UART-limited)  
✅ **No extra cable** (already plugged in)  
✅ **Zero CPU overhead** (ring buffer only)  
✅ **Code runs full speed** (not in debug mode)  

---

## 🚀 LAUNCH PROCEDURE

### Step 1: Build
```bash
cd /Users/yuanroets/stm32/nucleo-l476rg-blink/Meester_Playground
rm -rf build
cmake -B build
cmake --build build
```

### Step 2: Flash
- Use STM32 VS Code extension
- Keep ST-Link USB plugged in

### Step 3: Terminal 1 - Start RTT Capture
```bash
python3 tools/rtt_capture.py
```
You should see:
```
T,1234,512,100,-1050,45,-30,10
T,1240,520,105,-1045,48,-28,12
...
```

### Step 4: Terminal 2 - Start Plotter
```bash
cat /tmp/imu_stream | python3 tools/imu_live_plot_logger_stdin.py
```

Matplotlib window opens with live 6-axis plot!

---

## 💻 MAIN CODE PATTERNS EXPLAINED

### Pattern 1: Callback Registration

```c
/* Tell Bosch which functions to call */
bmi_dev.read = bmi2_i2c_read;       // When I need to read
bmi_dev.write = bmi2_i2c_write;     // When I need to write
bmi_dev.delay_us = bmi2_delay_us;   // When I need delay
bmi_dev.intf_ptr = &bmi2_i2c_ctx;   // Here's your context
```

**Why?** Bosch driver is generic. It calls function pointers you provide. You decide what those functions do!

### Pattern 2: Context Passing

```c
/* During initialization: store what you'll need later */
bmi2_i2c_ctx.hi2c = &hi2c1;
bmi2_i2c_ctx.dev_addr = 0x68;

/* During callback: retrieve what you stored */
bmi2_i2c_ctx_t *ctx = (bmi2_i2c_ctx_t *) intf_ptr;
ctx->hi2c;      // Use it here
ctx->dev_addr;  // Use it here
```

**Why?** Callbacks are called by external code (Bosch). You can't pass parameters directly. Use context pointer to communicate.

### Pattern 3: Non-blocking Timing

```c
/* Check if N milliseconds elapsed */
if ((HAL_GetTick() - last_imu_tick) >= 5U) {
  last_imu_tick = HAL_GetTick();  // Update timestamp
  // Do work here
}
```

**Why?** Never use `while()` loops in main. Check elapsed time, do work if ready, continue loop. Keeps CPU responsive to all tasks.

---

## 🎓 KEY LEARNING OUTCOMES

### What You Now Understand

1. **Pointers** - address storage, dereferencing, casting
2. **Callback pattern** - how external code calls your functions
3. **Opaque pointers** - how to pass custom context through callbacks
4. **Non-blocking scheduling** - how to run multiple tasks concurrently
5. **Data flow** - sensor → STM32 → RTT → Host → Plotter
6. **SEGGER RTT** - debug-speed data transfer over SWD
7. **I2C communication** - how BMI270 sensor talks to STM32

### Design Patterns Mastered

- ✅ Opaque pointer pattern (used in production everywhere)
- ✅ Callback registration pattern
- ✅ Ring buffer pattern (RTT)
- ✅ Non-blocking task scheduling
- ✅ State machine pattern (LED modes)

---

## 🔗 FILES MODIFIED

| File | Change |
|------|--------|
| `Core/Src/main.c` | Gyro enabled, RTT init, printf redirect |
| `CMakeLists.txt` | Added SEGGER_RTT compile |
| `third_party/SEGGER_RTT/` | RTT library (created) |
| `tools/rtt_capture.py` | RTT data capture (created) |
| `tools/RTT_SETUP.md` | Setup guide (created) |

---

## 📖 NEXT STEPS

1. **Build and test** the system on hardware
2. **Watch live plot** streaming at 200 Hz
3. **Understand LSB units** in plotter output
4. **Add barometer** (BMP390) on I2C2 using same pattern
5. **Implement FIFO interrupt** for more robust data capture

---

## 🎯 ONE-LINE SUMMARIES

| Concept | TL;DR |
|---------|-------|
| Pointers | Store addresses; `&` gets address, `*` gets value |
| Callbacks | External code calls your functions with context |
| Opaque pointers | Pass your custom data through generic API |
| RTT | Debug-speed printf output over SWD cable |
| Non-blocking | Check time, do work if ready, continue loop |
| Bosch pattern | Register callbacks, pass context, driver uses them |

---

## 📝 Files Worth Re-Reading

1. **`Core/Src/main.c`** - Lines 120-180 (callback functions)
2. **`Core/Src/main.c`** - Lines 220-280 (initialization flow)
3. **`tools/RTT_SETUP.md`** - Hardware setup guide

---

**End of Summary**

This represents a solid understanding of **embedded systems fundamentals**: callbacks, pointers, real-time I/O, and data streaming. These patterns appear in every production firmware project.

Great work learning this! 🎉
