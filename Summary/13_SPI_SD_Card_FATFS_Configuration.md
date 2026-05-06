# SPI SD Card + FATFS Configuration Guide (kiwih Method)

## Overview
This document explains the STM32L476RG + Nucleo board setup for SPI-based SD card access using kiwih's FatFS driver implementation. All pins and handles are now properly configured.

## Hardware Configuration

### SPI2 Pin Assignment
| Signal | STM32L476 Pin | Nucleo Pin | Direction |
|--------|---------------|-----------|-----------|
| SPI2_CLK | PB13 | CN7 28 | Output |
| SPI2_MOSI | PC3 | CN10 1 | Output |
| SPI2_MISO | PB14 | CN7 27 | Input |
| SD_CS (GPIO) | PC4 | CN10 34 | Output |

### CubeMX Configuration
- **SPI2 Mode**: Master, Full-duplex, 8-bit data
- **Initial Baudrate**: 256 prescaler (~312.5 kBit/s, slow for init)
- **NSS**: Software-controlled (manual GPIO management via PC4)
- **Polarity/Phase**: CPOL=0, CPHA=0

### GPIO Configuration
- **SD_CS (PC4)**: Output push-pull, pullup enabled, initialized HIGH
  - Controlled by macros `CS_HIGH()` / `CS_LOW()` in user_diskio_spi.c

## Firmware Configuration

### main.h Pin/Handle Definitions
```c
/* SD Card Configuration */
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOC
#define SD_SPI_HANDLE hspi2
```

### main.c Initialization Order
```c
1. MX_GPIO_Init()     /* Configure GPIO including SD_CS */
2. MX_SPI2_Init()     /* Configure SPI2 peripheral and pins */
3. MX_FATFS_Init()    /* Initialize FatFS layer */
```

### SPI Initialization (main.c - MX_SPI2_Init)
- Prescaler starts at 256 (slow ~312.5 kBit/s for SD card init)
- Switches to 8 (fast ~4.5 MBit/s) after card initialization
- Uses `MODIFY_REG()` macros for on-the-fly prescaler switching

## Critical Fix Applied

### Issue: Missing Include in user_diskio_spi.c
**Problem**: The driver couldn't access SD_SPI_HANDLE, SD_CS_Pin, and SD_CS_GPIO_Port macros defined in main.h.

**Root Cause**: `user_diskio_spi.c` did not include `main.h`, so preprocessor macro substitution failed:
- Code had: `extern SPI_HandleTypeDef SD_SPI_HANDLE;`
- main.h had: `#define SD_SPI_HANDLE hspi2`
- Without the include, linker searched for symbol "SD_SPI_HANDLE" instead of "hspi2"

**Solution**:
```c
#include "stm32l4xx_hal.h"
#include "main.h"              /* ← Added this line */
#include "user_diskio_spi.h"
```

This ensures:
1. `SD_SPI_HANDLE` macro resolves to `hspi2` at compile time
2. `SD_CS_GPIO_Port` and `SD_CS_Pin` macros are available
3. Proper linking to the actual handle variable in main.c

## FatFS Integration Chain
```
main.c (hspi2 handle)
    ↓
main.h (macro definitions)
    ↓
user_diskio_spi.c (includes main.h → accesses hspi2 via macro)
    ↓
user_diskio.c (wraps user_diskio_spi functions)
    ↓
fatfs.c / ff_gen_drv.h (FatFS middleware)
```

## Configuration Checklist
- [x] SPI2 peripheral configured in CubeMX
- [x] SPI2 pins mapped: PB13(CLK), PC3(MOSI), PB14(MISO)
- [x] CS pin configured as GPIO: PC4
- [x] main.h defines SD_SPI_HANDLE, SD_CS_Pin, SD_CS_GPIO_Port
- [x] main.c declares extern SPI_HandleTypeDef hspi2
- [x] user_diskio_spi.c includes main.h ✓ **(FIXED)**
- [x] Initialization order: GPIO → SPI2 → FATFS
- [x] CS pullup enabled in GPIO config

## Expected Behavior

### Card Initialization (slow clock)
1. CS goes LOW
2. 80 dummy clock cycles sent
3. CMD0 sequence executes (SD card enters idle state)
4. Card responds with R1 response byte
5. CMD8, ACMD41, CMD58 sequence completes card type detection

### Fast Mode Activation
Once card detected and initialized, prescaler switches from 256 → 8 (4× faster clock).

## Common Issues & Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Linker error: undefined reference to `SD_SPI_HANDLE` | Missing `#include "main.h"` in user_diskio_spi.c | Add include (now done) |
| CS pin not toggling | Missing GPIO init or wrong pin definition | Verify PC4 in GPIO_Init() |
| SPI not communicating | SPI handle not properly initialized | Check MX_SPI2_Init() called before FATFS init |
| Card always times out | Prescaler never switches to fast mode | Verify FCLK_FAST macro executes after init |

## Debugging Tips

1. **Verify handle linkage**:
   ```c
   // In main() or early init:
   if (hspi2.Instance != SPI2) {
       while(1); // Error: hspi2 not properly initialized
   }
   ```

2. **Monitor CS line** with oscilloscope/logic analyzer during card init

3. **Check prescaler switches** by reading SPI2->CR1 during operation

4. **Use RTT logging** to capture initialization sequence:
   ```c
   SEGGER_RTT_printf(0, "SPI Init: handle=%p\n", (void*)&SD_SPI_HANDLE);
   ```

## References
- kiwih FatFS SPI driver: Original by ChaN (elm-chan.org), ported by kiwih
- STM32L4 HAL SPI documentation
- FatFS generic diskio interface specification
