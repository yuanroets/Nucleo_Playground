/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bmi270.h"
#include "bmp3.h"
#include "SEGGER_RTT.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* CSV output period for serial plotting/logging tool.
 * 200 Hz sampling = 5 ms per sample. Print every sample for live plotting.
 */
#define IMU_VISUAL_REFRESH_MS 5U
/* Barometer task period: 25 Hz output is enough for altitude/temperature trend. */
#define BARO_VISUAL_REFRESH_MS 40U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Event flag set by EXTI callback when a valid (debounced) button press occurs. */
volatile uint8_t button_event = 0;
/* 0 = LED off, 1 = LED on, 2 = LED blink. */
volatile uint8_t led_mode = 0;

/* Tick snapshot used to schedule LED blink task. */
uint32_t last_blink_tick = 0;
/* Tick snapshot used for button debounce window. */
uint32_t last_button_tick = 0;
/* Tick snapshot used to schedule IMU polling task. */
uint32_t last_imu_tick = 0;
/* Tick snapshot used to schedule BMP390 polling task. */
uint32_t last_baro_tick = 0;
/* Tick snapshot used to print low-rate IMU health telemetry. */
uint32_t last_imu_stats_tick = 0;

/* Simple health counters for IMU polling path. */
uint32_t imu_read_ok_count = 0;
uint32_t imu_read_err_count = 0;
/* Simple health counters for barometer polling path. */
uint32_t baro_read_ok_count = 0;
uint32_t baro_read_err_count = 0;

/* Bosch driver state object (callbacks, interface mode, internal status). */
struct bmi2_dev bmi_dev;
/* Latest accel/gyro sample read from sensor; starts zeroed. */
struct bmi2_sens_data bmi_sens_data = { 0 };

/* Small context object passed into Bosch callbacks.
 * It tells callback code which I2C peripheral and sensor address to use.
 */
typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t dev_addr;
} bmi2_i2c_ctx_t;

bmi2_i2c_ctx_t bmi2_i2c_ctx;

/* Bosch BMP3 driver state object (BMP390 uses same BMP3 API). */
struct bmp3_dev bmp_dev;
/* Latest pressure/temperature sample from BMP390. */
struct bmp3_data bmp_data = { 0 };

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t dev_addr;
} bmp3_i2c_ctx_t;

bmp3_i2c_ctx_t bmp3_i2c_ctx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr,
                                            const uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr);
static void bmi2_delay_us(uint32_t period, void *intf_ptr);
static int8_t bmi270_basic_init(void);
static BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr,
                                         const uint8_t *reg_data,
                                         uint32_t len,
                                         void *intf_ptr);
static void bmp3_delay_us(uint32_t period, void *intf_ptr);
static uint8_t bmp390_detect_address(uint8_t *found_addr);
static int8_t bmp390_basic_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -------------------------------------------------------------------------
 * OWNERSHIP LEGEND (important for learning)
 * -------------------------------------------------------------------------
 * [OURS]      : Code we write and should understand deeply.
 * [DRIVER]    : External API call (Bosch or HAL). Use the function correctly;
 *               no need to know internal implementation right now.
 * [GENERATED] : CubeMX-generated setup code. Learn what each setting means,
 *               but treat structure/boilerplate as generated pattern.
 * ------------------------------------------------------------------------- */

/* Learning map for this file:
 * 1) Set up UART/I2C/GPIO peripherals.
 * 2) Configure BMI270 through Bosch driver callbacks.
 * 3) In the main loop, run simple non-blocking tasks (LED + IMU print).
 */

/* Redirect printf characters to SEGGER RTT so logs appear over debug link.
 * RTT sends data through SWD/ST-Link with zero performance overhead.
 * No UART needed for debug output.
 */
/* [OURS] This function is ours (retarget hook). */
/* [DRIVER] SEGGER_RTT_PutChar is a library API we use as a black box. */
int __io_putchar(int ch)
{
  /* RTT APIs work with single characters, so cast int to char. */
  char c = (char) ch;
  /* RTT buffer write is non-blocking and very fast. */
  SEGGER_RTT_PutChar(c);
  /* Return original character as stdio expects. */
  return ch;
}

/* Bosch read callback: read bytes from BMI270 register space via I2C. */
/* [OURS] Function is ours.
 * [DRIVER] Signature pattern is dictated by Bosch callback typedef.
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  /* Recover our context pointer that we stored in bmi_dev.intf_ptr. */
  // we have to do this because intf_ptr is void so we have to cast it 
  // back to our context type to get the I2C handle and address
  bmi2_i2c_ctx_t *ctx = (bmi2_i2c_ctx_t *) intf_ptr;
  /* HAL call result holder. */
  HAL_StatusTypeDef status;

  /* Simple defensive checks so we fail safely if context is invalid. */
  if ((ctx == NULL) || (ctx->hi2c == NULL) || (reg_data == NULL) || (len == 0U))
  {
    return (BMI2_INTF_RETURN_TYPE) -1;
  }

  /* HAL expects shifted address value in this API (7-bit addr << 1). */
  /* Read `len` bytes from register `reg_addr` into caller buffer `reg_data`. */
  /* [DRIVER] HAL_I2C_Mem_Read does the actual I2C transfer for us. */
  status = HAL_I2C_Mem_Read(ctx->hi2c,
                            (uint16_t) (ctx->dev_addr << 1U),
                            reg_addr,
                            I2C_MEMADD_SIZE_8BIT,
                            reg_data,
                            (uint16_t) len,
                            HAL_MAX_DELAY);

  /* Convert HAL result into Bosch expected return code convention. */
  return (status == HAL_OK) ? BMI2_INTF_RET_SUCCESS : (BMI2_INTF_RETURN_TYPE) -1;
}

/* Bosch write callback: write bytes to BMI270 register space via I2C. */
/* [OURS] Function is ours.
 * [DRIVER] Signature pattern is dictated by Bosch callback typedef.
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr,
                                            const uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr)
{
  /* Recover our context pointer that we stored in bmi_dev.intf_ptr. */
  bmi2_i2c_ctx_t *ctx = (bmi2_i2c_ctx_t *) intf_ptr;
  /* HAL call result holder. */
  HAL_StatusTypeDef status;

  /* Same guard checks as read path. */
  if ((ctx == NULL) || (ctx->hi2c == NULL) || (reg_data == NULL) || (len == 0U))
  {
    return (BMI2_INTF_RETURN_TYPE) -1;
  }

  /* HAL expects shifted address value in this API (7-bit addr << 1). */
  /* Write `len` bytes from `reg_data` into sensor register `reg_addr`. */
  /* [DRIVER] HAL_I2C_Mem_Write does the actual I2C transfer for us. */
  status = HAL_I2C_Mem_Write(ctx->hi2c,
                             (uint16_t) (ctx->dev_addr << 1U),
                             reg_addr,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *) reg_data,
                             (uint16_t) len,
                             HAL_MAX_DELAY);

  /* Convert HAL result into Bosch expected return code convention. */
  return (status == HAL_OK) ? BMI2_INTF_RET_SUCCESS : (BMI2_INTF_RETURN_TYPE) -1;
}

/* Bosch delay callback.
 * For now we use a simple busy-wait, which is fine for early learning/bring-up.
 */
/* [OURS] Function is ours.
 * [DRIVER] Callback hook is required by Bosch API pattern.
 */
static void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
  /* Volatile prevents compiler from optimizing loop away. */
  volatile uint32_t cycles;
  /* Loop counter. */
  uint32_t i;

  /* Unused in this implementation, required by callback signature. */
  (void) intf_ptr;

  /* Convert microseconds to rough loop cycles using current core clock. */
  cycles = (SystemCoreClock / 4000000U) * period;
  /* Busy-wait loop: simple and okay for first-stage bring-up. */
  for (i = 0; i < cycles; i++)
  {
    /* NOP keeps one predictable instruction per iteration. */
    __NOP();
  }
}

/* Bosch BMP3 read callback: read bytes from BMP390 register space via I2C. */
static BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  bmp3_i2c_ctx_t *ctx = (bmp3_i2c_ctx_t *) intf_ptr;
  HAL_StatusTypeDef status;

  if ((ctx == NULL) || (ctx->hi2c == NULL) || (reg_data == NULL) || (len == 0U))
  {
    return (BMP3_INTF_RET_TYPE) -1;
  }

  status = HAL_I2C_Mem_Read(ctx->hi2c,
                            (uint16_t) (ctx->dev_addr << 1U),
                            reg_addr,
                            I2C_MEMADD_SIZE_8BIT,
                            reg_data,
                            (uint16_t) len,
                            HAL_MAX_DELAY);

  return (status == HAL_OK) ? BMP3_INTF_RET_SUCCESS : (BMP3_INTF_RET_TYPE) -1;
}

/* Bosch BMP3 write callback: write bytes to BMP390 register space via I2C. */
static BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr,
                                         const uint8_t *reg_data,
                                         uint32_t len,
                                         void *intf_ptr)
{
  bmp3_i2c_ctx_t *ctx = (bmp3_i2c_ctx_t *) intf_ptr;
  HAL_StatusTypeDef status;

  if ((ctx == NULL) || (ctx->hi2c == NULL) || (reg_data == NULL) || (len == 0U))
  {
    return (BMP3_INTF_RET_TYPE) -1;
  }

  status = HAL_I2C_Mem_Write(ctx->hi2c,
                             (uint16_t) (ctx->dev_addr << 1U),
                             reg_addr,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *) reg_data,
                             (uint16_t) len,
                             HAL_MAX_DELAY);

  return (status == HAL_OK) ? BMP3_INTF_RET_SUCCESS : (BMP3_INTF_RET_TYPE) -1;
}

/* BMP3 delay callback implemented with same busy-wait style as BMI270 path. */
static void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
  bmi2_delay_us(period, intf_ptr);
}

/* Probe both BMP390 I2C addresses so board strap variants work without code edits. */
static uint8_t bmp390_detect_address(uint8_t *found_addr)
{
  uint8_t chip_id;
  HAL_StatusTypeDef status;
  uint8_t i;
  const uint8_t candidate_addrs[2] = { BMP3_ADDR_I2C_SEC, BMP3_ADDR_I2C_PRIM };

  if (found_addr == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < 2U; i++)
  {
    status = HAL_I2C_Mem_Read(&hi2c2,
                              (uint16_t) (candidate_addrs[i] << 1U),
                              BMP3_REG_CHIP_ID,
                              I2C_MEMADD_SIZE_8BIT,
                              &chip_id,
                              1U,
                              50U);
    if ((status == HAL_OK) && ((chip_id == BMP390_CHIP_ID) || (chip_id == BMP3_CHIP_ID)))
    {
      *found_addr = candidate_addrs[i];
      return 1U;
    }
  }

  return 0U;
}

/* Full BMP390 setup: pressure + temperature in normal mode over I2C2. */
static int8_t bmp390_basic_init(void)
{
  int8_t rslt;
  uint8_t detected_addr;
  uint16_t desired_settings;
  struct bmp3_settings bmp_settings;

  if (!bmp390_detect_address(&detected_addr))
  {
    printf("BMP390 not detected on hi2c2 (tried 0x77 and 0x76)\r\n");
    return (int8_t) -1;
  }

  bmp3_i2c_ctx.hi2c = &hi2c2;
  bmp3_i2c_ctx.dev_addr = detected_addr;

  bmp_dev.intf = BMP3_I2C_INTF;
  bmp_dev.read = bmp3_i2c_read;
  bmp_dev.write = bmp3_i2c_write;
  bmp_dev.delay_us = bmp3_delay_us;
  bmp_dev.intf_ptr = &bmp3_i2c_ctx;

  rslt = bmp3_init(&bmp_dev);
  if (rslt != BMP3_OK)
  {
    printf("BMP390 init failed: %d\r\n", rslt);
    return rslt;
  }

  rslt = bmp3_get_sensor_settings(&bmp_settings, &bmp_dev);
  if (rslt != BMP3_OK)
  {
    printf("BMP390 get cfg failed: %d\r\n", rslt);
    return rslt;
  }

  bmp_settings.press_en = BMP3_ENABLE;
  bmp_settings.temp_en = BMP3_ENABLE;
  bmp_settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
  bmp_settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
  bmp_settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
  bmp_settings.odr_filter.odr = BMP3_ODR_25_HZ;

  desired_settings = BMP3_SEL_PRESS_EN |
                     BMP3_SEL_TEMP_EN |
                     BMP3_SEL_PRESS_OS |
                     BMP3_SEL_TEMP_OS |
                     BMP3_SEL_IIR_FILTER |
                     BMP3_SEL_ODR;

  rslt = bmp3_set_sensor_settings((uint32_t) desired_settings, &bmp_settings, &bmp_dev);
  if (rslt != BMP3_OK)
  {
    printf("BMP390 set cfg failed: %d\r\n", rslt);
    return rslt;
  }

  bmp_settings.op_mode = BMP3_MODE_NORMAL;
  rslt = bmp3_set_op_mode(&bmp_settings, &bmp_dev);
  if (rslt != BMP3_OK)
  {
    printf("BMP390 set mode failed: %d\r\n", rslt);
    return rslt;
  }

  printf("BMP390 ready, addr:0x%02X chip id:0x%02X\r\n", (unsigned int) detected_addr, (unsigned int) bmp_dev.chip_id);
  return BMP3_OK;
}

/* Full BMI270 setup: accel + gyro at 200Hz for athlete monitoring. */
/* [OURS] This orchestration function is ours.
 * [DRIVER] Inside it, Bosch API calls are used as black-box operations.
 */
static int8_t bmi270_basic_init(void)
{
  /* Bosch return code variable (0 usually means success). */
  int8_t rslt;
  /* Config structures for both accel and gyro settings. */
  struct bmi2_sens_config accel_cfg;
  struct bmi2_sens_config gyro_cfg;
  /* Sensor list: enable both accel and gyro for full 6-axis (3a+3g) data. */
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  /* Store which hardware I2C instance and address Bosch should use. */
  // remember *hi2c in struct (ie pointer)
  // so this one is kind of our hardcoded variable
  bmi2_i2c_ctx.hi2c = &hi2c1;
  /* Primary BMI270 I2C address (normally 0x68 depending on board jumper). */
  bmi2_i2c_ctx.dev_addr = BMI2_I2C_PRIM_ADDR;

  /* Plug our callbacks/context into Bosch driver object. */
  /* [OURS] Assign callback pointers and context data. */
  bmi_dev.intf = BMI2_I2C_INTF;
  /* Function pointer called by Bosch when driver needs register reads. */
  bmi_dev.read = bmi2_i2c_read;
  /* Function pointer called by Bosch when driver needs register writes. */
  bmi_dev.write = bmi2_i2c_write;
  /* Function pointer called by Bosch when delays are needed. */
  bmi_dev.delay_us = bmi2_delay_us;
  /* Opaque pointer passed back into our callbacks. */
  // this one is just so that we can pass it to the driver in a nice package
  bmi_dev.intf_ptr = &bmi2_i2c_ctx;
  /* Max chunk size for a single transfer (safe/simple value). */
  bmi_dev.read_write_len = 32;

  /* Bring up BMI270 internal state and validate communication. */
  /* [DRIVER] Bosch API call: initializes chip and internal driver state. */
  // we have to 'activate' the driver by calling this init function, 
  // which will do a chip ID read and some internal setup
  rslt = bmi270_init(&bmi_dev);
  if (rslt != BMI2_OK)
  {
    printf("BMI270 init failed: %d\r\n", rslt);
    return rslt;
  }

  /* Read-modify-write config flow keeps setup explicit and easy to follow. */
  // since everything now active, we can use the funciton from the driver
  /* [DRIVER] Bosch API call: read current accel config. */
  accel_cfg.type = BMI2_ACCEL;
  rslt = bmi2_get_sensor_config(&accel_cfg, 1, &bmi_dev);
  if (rslt != BMI2_OK)
  {
    printf("BMI270 get accel cfg failed: %d\r\n", rslt);
    return rslt;
  }

  /* Accelerometer Configuration for athlete monitoring (lumbar mount). */
  /* 200 Hz ODR captures running stride dynamics (~1.5-3 Hz fundamental). */
  accel_cfg.cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  /* ±16g range captures high-impact accelerations (falls, jumps, hard landings). */
  accel_cfg.cfg.acc.range = BMI2_ACC_RANGE_16G;
  /* OSR2 averaging provides smooth filtering without excessive lag. */
  accel_cfg.cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  /* Performance mode optimizes for power and accuracy balance. */
  accel_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  /* Write accel config back to the sensor. */
  /* [DRIVER] Bosch API call: apply accel config. */
  rslt = bmi2_set_sensor_config(&accel_cfg, 1, &bmi_dev);
  if (rslt != BMI2_OK)
  {
    printf("BMI270 set accel cfg failed: %d\r\n", rslt);
    return rslt;
  }

  /* Gyroscope Configuration for rotational motion during running. */
  /* 200 Hz ODR captures angular velocity at same rate as accel for sync. */
  gyro_cfg.type = BMI2_GYRO;
  rslt = bmi2_get_sensor_config(&gyro_cfg, 1, &bmi_dev);
  if (rslt != BMI2_OK)
  {
    printf("BMI270 get gyro cfg failed: %d\r\n", rslt);
    return rslt;
  }

  /* 200 Hz matches accel for synchronized 6-axis data logging. */
  gyro_cfg.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  /* ±2000 dps range captures body rotation (pitch, roll, yaw) in running. */
  gyro_cfg.cfg.gyr.range = BMI2_GYR_RANGE_2000;
  /* OSR2 filtering mirrors accel config for consistent data quality. */
  gyro_cfg.cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  /* Performance mode for gyro as well. */
  gyro_cfg.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  /* Write gyro config back to the sensor. */
  /* [DRIVER] Bosch API call: apply gyro config. */
  rslt = bmi2_set_sensor_config(&gyro_cfg, 1, &bmi_dev);
  if (rslt != BMI2_OK)
  {
    printf("BMI270 set gyro cfg failed: %d\r\n", rslt);
    return rslt;
  }

  /* Enable both accel and gyro data paths so reads return valid 6-axis samples. */
  /* [DRIVER] Bosch API call: enable selected sensor(s). */
  rslt = bmi2_sensor_enable(sens_list, 2, &bmi_dev);
  if (rslt != BMI2_OK)
  {
    printf("BMI270 accel enable failed: %d\r\n", rslt);
    return rslt;
  }

  printf("BMI270 ready, chip id: 0x%02X\r\n", bmi_dev.chip_id);
  /* Report success to caller. */
  return BMI2_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* STEP A: Initialize SEGGER RTT for debug output over SWD/ST-Link.
   * This must happen before any printf() calls.
   * RTT uses the debug connection (no UART needed for debug output).
   */
  SEGGER_RTT_Init();

  /* STEP D: first visible debug print. If you see this in RTT, CPU + debug link are alive. */
  printf("Booting...\r\n");

  /* STEP E: initialize sensor stack (callbacks + Bosch init + accel config). */
  /* Stop here if sensor init fails so failure is obvious during bring-up. */
  /* [OURS] We choose when init happens and what to do on failure. */
  if (bmi270_basic_init() != BMI2_OK)
  {
    Error_Handler();
  }

  /* STEP E2: initialize BMP390 (Bosch BMP3 API) on I2C2 for pressure + temperature. */
  if (bmp390_basic_init() != BMP3_OK)
  {
    Error_Handler();
  }

  /* STEP F: set known initial state before entering endless loop.
   * Setting baseline ticks now avoids an immediate first-loop trigger.
   */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  last_blink_tick = HAL_GetTick();
  last_imu_tick = HAL_GetTick();
  last_baro_tick = HAL_GetTick();
  last_imu_stats_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* TASK 1: process button events captured by ISR. */
    /* Button ISR sets button_event; main loop handles behavior change. */
    /* [OURS] This whole control logic is yours to understand deeply. */
    if (button_event)
    {
      /* Consume the event once, then clear it.
       * Event is set in EXTI callback after debounce.
       */
      button_event = 0;
      /* Cycle through 0 -> 1 -> 2 -> 0 ... */
      led_mode = (uint8_t)((led_mode + 1U) % 3U);

      /* Mode 0: LED off, mode 1: LED on, mode 2: LED blinks. */
      if (led_mode == 0U)
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }
      else if (led_mode == 1U)
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      }
      else
      {
        /* Reset blink timer when entering blink mode for clean phase start. */
        last_blink_tick = HAL_GetTick();
      }
    }

    /* TASK 2: blink LED periodically when mode == 2. */
    /* Non-blocking blink task: only active in mode 2. */
    /* [OURS] Timing/state pattern is yours. */
    if (led_mode == 2U)
    {
      /* Read current time once per iteration for this task. */
      uint32_t now = HAL_GetTick();
      /* If 200 ms elapsed, perform one toggle action. */
      if ((now - last_blink_tick) >= 200U)
      {
        /* Save timestamp before toggle so interval stays stable over time. */
        last_blink_tick = now;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
    }

    /* TASK 3: periodically read full 6-axis IMU data (accel + gyro) and print. */
    /* 5ms interval = 200 Hz sampling matches BMI270 ODR for complete data capture. */
    /* This output feeds directly to live_plotter.py for real-time visualization. */
    /* [OURS] Scheduling and response handling are ours.
     * [DRIVER] Actual sensor transfer/parse is Bosch + HAL.
     */
    if ((HAL_GetTick() - last_imu_tick) >= IMU_VISUAL_REFRESH_MS)
    {
      /* Local result variable for this transaction. */
      int8_t rslt;

      /* Same timing pattern as blink task: do work only when period elapsed. */
      last_imu_tick = HAL_GetTick();
      /* Bosch API fills bmi_sens_data with accel(x,y,z) and gyro(x,y,z) values. */
      /* bmi_sens_data.acc.x, .y, .z = accelerometer (3-axis, LSB units, ±16g). */
      /* bmi_sens_data.gyr.x, .y, .z = gyroscope (3-axis, LSB units, ±2000 dps). */
      /* [DRIVER] Bosch high-level read API polls sensor via I2C callbacks. */
      rslt = bmi2_get_sensor_data(&bmi_sens_data, &bmi_dev);
      if (rslt == BMI2_OK)
      {
        imu_read_ok_count++;
        /* CSV format: T,t_ms,ax,ay,az,gx,gy,gz
         * T = marker (fixed string)
         * t_ms = timestamp from STM32 HAL_GetTick() in milliseconds
         * ax,ay,az = accelerometer raw LSB values (±16g mapped to ~±32768 LSB)
         * gx,gy,gz = gyroscope raw LSB values (±2000 dps mapped to ~±32768 LSB)
         * This format matches imu_live_plot_logger.py expectations.
         */
        printf("T,%lu,%d,%d,%d,%d,%d,%d\r\n",
          (unsigned long) HAL_GetTick(),
          (int) bmi_sens_data.acc.x,
          (int) bmi_sens_data.acc.y,
          (int) bmi_sens_data.acc.z,
          (int) bmi_sens_data.gyr.x,
          (int) bmi_sens_data.gyr.y,
          (int) bmi_sens_data.gyr.z);
      }
      else
      {
        imu_read_err_count++;
        /* Keep error visible; useful when wiring/address is wrong. */
        printf("BMI270 read failed: %d\r\n", rslt);
      }
    }

    /* TASK 4: periodically read BMP390 pressure+temperature and print compact CSV. */
    if ((HAL_GetTick() - last_baro_tick) >= BARO_VISUAL_REFRESH_MS)
    {
      int8_t rslt;

      last_baro_tick = HAL_GetTick();
      rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp_data, &bmp_dev);
      if (rslt == BMP3_OK)
      {
        int32_t pressure_pa = (int32_t) (bmp_data.pressure);
        int32_t temp_c_x100 = (int32_t) (bmp_data.temperature * 100.0);

        baro_read_ok_count++;
        /* CSV format: P,t_ms,pressure_pa,temp_c_x100 */
        printf("P,%lu,%ld,%ld\r\n",
               (unsigned long) HAL_GetTick(),
               (long) pressure_pa,
               (long) temp_c_x100);
      }
      else
      {
        baro_read_err_count++;
        printf("BMP390 read failed: %d\r\n", rslt);
      }
    }

    /* TASK 5: print low-rate sensor health summary once per second. */
    if ((HAL_GetTick() - last_imu_stats_tick) >= 1000U)
    {
      last_imu_stats_tick = HAL_GetTick();
      printf("\r\nIMU ok:%lu err:%lu | BARO ok:%lu err:%lu\r\n",
             (unsigned long) imu_read_ok_count,
             (unsigned long) imu_read_err_count,
             (unsigned long) baro_read_ok_count,
             (unsigned long) baro_read_err_count);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F12981;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Repeated enable/priority calls are harmless but redundant.
   * Kept here to avoid changing behavior in this learning phase.
   */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* EXTI callback runs when configured GPIO interrupt fires.
 * Keep ISR work minimal: just debounce + set event flag.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* [OURS] ISR callback body is ours (called by HAL IRQ handler). */
  /* Make sure interrupt came from the expected button pin. */
  if (GPIO_Pin == Button_Pin)
  {
    /* Capture current time for debounce calculation. */
    uint32_t now = HAL_GetTick();
    /* Debounce: ignore rapid re-triggers within 150 ms. */
    if ((now - last_button_tick) >= 150U)
    {
      /* Save accepted press time. */
      last_button_tick = now;
      /* Signal main loop to handle the real behavior change. */
      button_event = 1;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* [OURS] Your project-level policy for fatal errors. */
  /* Disable interrupts and stay here forever.
   * In early bring-up this is useful because failure is obvious and repeatable.
   */
  __disable_irq();
  /* Trap CPU forever so fault state is stable and debuggable. */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
