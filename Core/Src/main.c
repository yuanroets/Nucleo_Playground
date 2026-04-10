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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "bmi270.h"
#include "bmp3.h"
#include "SEGGER_RTT.h"
#include "u_ubx_protocol.h"
#include "u_error_common.h"

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
/* GPS logging cadence (1 Hz) for dedicated GPS data file. */
#define GPS_LOG_REFRESH_MS 1000U
/* Flush all open CSV files from one central task in the main loop. */
#define DATA_LOG_SYNC_MS 2000U
/* Never block forever on I2C transfers; fail fast and recover next cycle. */
#define I2C_XFER_TIMEOUT_MS 10U
/* Verbose GPS bring-up logging: print raw chunks and every full NMEA line. */
#define GPS_EXTENSIVE_DEBUG 0U
/* UART RX DMA buffer used for GPS NMEA capture. */
#define GPS_RX_DMA_BUFFER_SIZE 256U
/* Line buffer for one NMEA sentence. */
#define GPS_NMEA_LINE_BUFFER_SIZE 128U
/* UBX stream scratch buffer used with official u-blox UBX protocol decode. */
#define GPS_UBX_STREAM_BUFFER_SIZE 512U
/* UBX message body scratch buffer. */
#define GPS_UBX_BODY_BUFFER_SIZE 128U
/* Temporary GPS bring-up diagnostics: keep this small so RTT stays readable. */
#define GPS_UART_DEBUG_BUDGET 3U
/* BMI270 accel raw output at +/-16g is 2048 LSB per g. */
#define BMI270_ACC_LSB_PER_G 2048.0
/* BMI270 gyro raw output at +/-2000 dps is 16.384 LSB per dps. */
#define BMI270_GYR_LSB_PER_DPS_2000 16.384

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Event flag set by EXTI callback when a valid (debounced) button press occurs. */
volatile uint8_t button_event = 0;
/* 0 = LED off, 1 = LED on, 2 = LED blink. */
volatile uint8_t logging_active = 0;

typedef struct
{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t milliseconds;
} SystemTime_t;

/* Tick snapshot used to schedule LED blink task. */
uint32_t last_blink_tick = 0;
/* Tick snapshot used for button debounce window. */
uint32_t last_button_tick = 0;
/* Tick snapshot used to schedule IMU polling task. */
uint32_t last_imu_tick = 0;
/* Tick snapshot used to schedule BMP390 polling task. */
uint32_t last_baro_tick = 0;
/* Tick snapshot used to schedule 1 Hz GPS logging task. */
uint32_t last_gps_log_tick = 0;
/* Tick snapshot used to print low-rate IMU health telemetry. */
uint32_t last_imu_stats_tick = 0;

/* Simple health counters for IMU polling path. */
uint32_t imu_read_ok_count = 0;
uint32_t imu_read_err_count = 0;
/* Simple health counters for barometer polling path. */
uint32_t baro_read_ok_count = 0;
uint32_t baro_read_err_count = 0;
/* Latest barometer sample, printed at 1 Hz to reduce RTT clutter. */
int32_t baro_last_pressure_pa = 0;
int32_t baro_last_temp_c_x100 = 0;
uint8_t baro_last_valid = 0U;

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

/* Monotonic system time used for CSV logging. When GPS is present, PPS locks this clock to UTC. */
static SystemTime_t system_time = { 0U, 0U, 0U, 0U };
static uint32_t system_time_last_tick = 0U;
static uint8_t system_time_initialized = 0U;
static uint8_t system_time_locked = 0U;
static uint8_t system_time_lock_reported = 0U;

/* SD logging file handles split by sensor group. */
static FIL imu_log_file;
static FIL baro_log_file;
static FIL gps_log_file;
static uint8_t data_log_ready = 0U;
static uint32_t data_log_last_sync_tick = 0U;

/* High-resolution PPS sync point captured from the GPS PPS interrupt.
 * DWT cycle counter is much finer than HAL_GetTick() and is suitable for drift correction.
 */
volatile uint32_t gps_pps_cycle_stamp = 0U;
volatile uint32_t gps_pps_tick_stamp = 0U;
volatile uint8_t gps_pps_sync_ready = 0U;

/* Circular DMA receive buffer filled by USART1. */
static uint8_t gps_rx_dma_buffer[GPS_RX_DMA_BUFFER_SIZE];
/* Read index into the circular DMA buffer. */
static uint16_t gps_rx_old_pos = 0U;
/* Line assembly buffer for one NMEA sentence. */
static char gps_line_buffer[GPS_NMEA_LINE_BUFFER_SIZE];
/* Current length of the line assembly buffer. */
static uint16_t gps_line_length = 0U;
static volatile uint32_t gps_uart_rx_event_count = 0U;
static volatile uint32_t gps_uart_rx_byte_count = 0U;
static volatile uint16_t gps_uart_last_chunk_size = 0U;
static volatile uint8_t gps_uart_started_ok = 0U;
static volatile uint32_t gps_uart_error_code = 0U;
static volatile uint16_t gps_uart_last_line_length = 0U;
static char gps_uart_last_line[GPS_NMEA_LINE_BUFFER_SIZE];
static uint8_t gps_ubx_stream_buffer[GPS_UBX_STREAM_BUFFER_SIZE];
static size_t gps_ubx_stream_length = 0U;
static char gps_ubx_body_buffer[GPS_UBX_BODY_BUFFER_SIZE];

typedef struct
{
  char utc[16];
  uint8_t fix_quality;
  uint8_t satellites;
  double latitude_deg;
  double longitude_deg;
  double altitude_m;
  uint8_t valid;
} gps_status_t;

static gps_status_t gps_status = { { 0 }, 0U, 0U, 0U };
static volatile uint8_t gps_status_dirty = 0U;

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
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
static void gps_uart_start_reception(void);
static void gps_uart_process_dma_chunk(uint16_t new_pos);
static void gps_uart_process_byte(uint8_t byte);
static void gps_uart_poll_dma(void);
static void gps_uart_report_debug(void);
static void gps_uart_dump_chunk(const uint8_t *data, uint16_t len);
static void gps_ubx_feed_byte(uint8_t byte);
static void gps_ubx_parse_nav_pvt(const char *body, int32_t body_len);
static void gps_emit_status(void);
static void system_time_advance_to_tick(uint32_t current_tick);
static void system_time_set_from_utc(const char *utc);
static void system_time_handle_pps(uint32_t pps_tick);
static void system_time_format(const SystemTime_t *time_value, char *buffer, size_t buffer_len);
static int32_t div_round_s32(int64_t numerator, int32_t denominator);
static void format_fixed_decimal(char *buffer,
                                 size_t buffer_len,
                                 int32_t value,
                                 uint32_t scale,
                                 uint8_t frac_digits);
static FRESULT data_log_start(void);
static void data_log_stop(void);
static void gps_copy_status_snapshot(gps_status_t *snapshot);
static void csv_log_imu_sample(const struct bmi2_sens_data *imu_sample,
                               const gps_status_t *gps_sample,
                               uint32_t log_tick);
static void csv_log_baro_sample(const struct bmp3_data *baro_sample,
                                const gps_status_t *gps_sample,
                                uint32_t log_tick);
static void csv_log_gps_sample(const gps_status_t *gps_sample,
                               uint32_t log_tick);
static uint8_t gps_parse_gga_sentence(const char *line);
static uint8_t gps_validate_checksum(const char *line);
static const char *gps_skip_field(const char *field);
static uint8_t gps_parse_uint8_field(const char *start, const char *end, uint8_t *value);
static uint8_t gps_parse_nmea_coordinate(const char *start, const char *end, char hemisphere, double *value);
static uint8_t gps_format_utc_field(const char *start, size_t length, char *out, size_t out_len);
static int8_t gps_hex_to_nibble(char c);
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

static void system_time_format(const SystemTime_t *time_value, char *buffer, size_t buffer_len)
{
  if ((time_value == NULL) || (buffer == NULL) || (buffer_len == 0U))
  {
    return;
  }

  (void) snprintf(buffer,
                  buffer_len,
                  "%02u:%02u:%02u.%03u",
                  (unsigned int) time_value->hours,
                  (unsigned int) time_value->minutes,
                  (unsigned int) time_value->seconds,
                  (unsigned int) time_value->milliseconds);
}

static void system_time_advance_to_tick(uint32_t current_tick)
{
  uint32_t delta_ms;
  uint32_t total_ms;
  uint32_t total_seconds;

  if (system_time_initialized == 0U)
  {
    system_time_last_tick = current_tick;
    system_time_initialized = 1U;
    return;
  }

  delta_ms = current_tick - system_time_last_tick;
  if (delta_ms == 0U)
  {
    return;
  }

  system_time_last_tick = current_tick;
  total_ms = delta_ms + (uint32_t) system_time.milliseconds;
  total_seconds = (uint32_t) system_time.seconds + (total_ms / 1000U);
  system_time.milliseconds = (uint16_t) (total_ms % 1000U);
  system_time.seconds = (uint8_t) (total_seconds % 60U);

  total_seconds = (uint32_t) system_time.minutes + (total_seconds / 60U);
  system_time.minutes = (uint8_t) (total_seconds % 60U);

  total_seconds = (uint32_t) system_time.hours + (total_seconds / 60U);
  system_time.hours = (uint8_t) (total_seconds % 24U);
}

static void system_time_set_from_utc(const char *utc)
{
  if ((utc == NULL) || (strlen(utc) < 8U))
  {
    return;
  }

  if ((utc[0] < '0') || (utc[0] > '9') ||
      (utc[1] < '0') || (utc[1] > '9') ||
      (utc[3] < '0') || (utc[3] > '9') ||
      (utc[4] < '0') || (utc[4] > '9') ||
      (utc[6] < '0') || (utc[6] > '9') ||
      (utc[7] < '0') || (utc[7] > '9'))
  {
    return;
  }

  system_time.hours = (uint8_t) ((uint8_t) (utc[0] - '0') * 10U + (uint8_t) (utc[1] - '0'));
  system_time.minutes = (uint8_t) ((uint8_t) (utc[3] - '0') * 10U + (uint8_t) (utc[4] - '0'));
  system_time.seconds = (uint8_t) ((uint8_t) (utc[6] - '0') * 10U + (uint8_t) (utc[7] - '0'));
  system_time.milliseconds = 0U;
  system_time_last_tick = HAL_GetTick();
  system_time_initialized = 1U;
}

static void system_time_handle_pps(uint32_t pps_tick)
{
  uint8_t should_report_locked = 0U;

  system_time_advance_to_tick(pps_tick);
  system_time.milliseconds = 0U;
  system_time.seconds++;
  if (system_time.seconds >= 60U)
  {
    system_time.seconds = 0U;
    system_time.minutes++;
    if (system_time.minutes >= 60U)
    {
      system_time.minutes = 0U;
      system_time.hours = (uint8_t) ((system_time.hours + 1U) % 24U);
    }
  }

  system_time_last_tick = pps_tick;
  system_time_initialized = 1U;

  if (system_time_locked == 0U)
  {
    system_time_locked = 1U;
    should_report_locked = 1U;
  }

  if ((should_report_locked != 0U) && (system_time_lock_reported == 0U))
  {
    system_time_lock_reported = 1U;
    SEGGER_RTT_printf("SYSTEM LOCKED\r\n");
  }
}

static FRESULT data_log_start(void)
{
  FRESULT result;

  result = f_mount(&USERFatFS, USERPath, 1);
  if (result == FR_NO_FILESYSTEM)
  {
    SEGGER_RTT_printf("SDLOG|mount failed: %d (no FAT filesystem)\r\n", (int) result);
    SEGGER_RTT_printf("SDLOG|format card as FAT32 on PC, then press button again\r\n");
    return result;
  }

  if (result == FR_DISK_ERR)
  {
    const unsigned int sd_stage = (unsigned int) USER_SD_GetDiagStage();
    const unsigned int sd_cmd = (unsigned int) USER_SD_GetDiagLastCmd();
    const unsigned int sd_r1 = (unsigned int) USER_SD_GetDiagLastR1();
    const unsigned int sd_token = (unsigned int) USER_SD_GetDiagLastToken();
    const unsigned int sd_card_type = (unsigned int) USER_SD_GetCardType();

    SEGGER_RTT_printf("SDLOG|mount failed: %d (disk I/O error)\r\n", (int) result);
    SEGGER_RTT_printf("SDLOG|check SD SPI wiring/power and card seat\r\n");
    SEGGER_RTT_printf("SDLOG|diag stage=0x%02X cmd=%u r1=0x%02X token=0x%02X ctype=0x%02X\r\n",
                      sd_stage,
                      sd_cmd,
                      sd_r1,
                      sd_token,
                      sd_card_type);
    return result;
  }

  if (result != FR_OK)
  {
    SEGGER_RTT_printf("SDLOG|mount failed: %d\r\n", (int) result);
    return result;
  }

  result = f_open(&imu_log_file, "0:/IMU_DATA.CSV", FA_CREATE_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    SEGGER_RTT_printf("SDLOG|open IMU_DATA.CSV failed: %d\r\n", (int) result);
    return result;
  }

  result = f_open(&baro_log_file, "0:/BARO_DATA.CSV", FA_CREATE_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    (void) f_close(&imu_log_file);
    SEGGER_RTT_printf("SDLOG|open BARO_DATA.CSV failed: %d\r\n", (int) result);
    return result;
  }

  result = f_open(&gps_log_file, "0:/GPS_DATA.CSV", FA_CREATE_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    (void) f_close(&baro_log_file);
    (void) f_close(&imu_log_file);
    SEGGER_RTT_printf("SDLOG|open GPS_DATA.CSV failed: %d\r\n", (int) result);
    return result;
  }

  if (f_printf(&imu_log_file,
               "Sys_Time,UTC_GPS,AccX_g,AccY_g,AccZ_g,GyroX_dps,GyroY_dps,GyroZ_dps\r\n") < 0)
  {
    (void) f_close(&gps_log_file);
    (void) f_close(&baro_log_file);
    (void) f_close(&imu_log_file);
    SEGGER_RTT_printf("SDLOG|IMU header write failed\r\n");
    return FR_DISK_ERR;
  }

  if (f_printf(&baro_log_file,
               "Sys_Time,UTC_GPS,Temp_C,Press_kPa\r\n") < 0)
  {
    (void) f_close(&gps_log_file);
    (void) f_close(&baro_log_file);
    (void) f_close(&imu_log_file);
    SEGGER_RTT_printf("SDLOG|BARO header write failed\r\n");
    return FR_DISK_ERR;
  }

  if (f_printf(&gps_log_file,
               "Sys_Time,UTC_GPS,Lat_deg,Lon_deg,Alt_m,Sats\r\n") < 0)
  {
    (void) f_close(&gps_log_file);
    (void) f_close(&baro_log_file);
    (void) f_close(&imu_log_file);
    SEGGER_RTT_printf("SDLOG|GPS header write failed\r\n");
    return FR_DISK_ERR;
  }

  data_log_ready = 1U;
  data_log_last_sync_tick = HAL_GetTick();
  (void) f_sync(&imu_log_file);
  (void) f_sync(&baro_log_file);
  (void) f_sync(&gps_log_file);
  SEGGER_RTT_printf("SDLOG|logging started\r\n");
  return FR_OK;
}

static void data_log_stop(void)
{
  if (data_log_ready == 0U)
  {
    return;
  }

  (void) f_sync(&imu_log_file);
  (void) f_sync(&baro_log_file);
  (void) f_sync(&gps_log_file);
  (void) f_close(&imu_log_file);
  (void) f_close(&baro_log_file);
  (void) f_close(&gps_log_file);
  data_log_ready = 0U;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  SEGGER_RTT_printf("SDLOG|logging stopped\r\n");
}

static void gps_copy_status_snapshot(gps_status_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  __disable_irq();
  *snapshot = gps_status;
  __enable_irq();
}

static int32_t div_round_s32(int64_t numerator, int32_t denominator)
{
  if (denominator == 0)
  {
    return 0;
  }

  if (numerator >= 0)
  {
    return (int32_t) ((numerator + (denominator / 2)) / denominator);
  }

  return (int32_t) ((numerator - (denominator / 2)) / denominator);
}

static void format_fixed_decimal(char *buffer,
                                 size_t buffer_len,
                                 int32_t value,
                                 uint32_t scale,
                                 uint8_t frac_digits)
{
  int64_t abs_value;
  long int_part;
  long frac_part;
  const char *sign;

  if ((buffer == NULL) || (buffer_len == 0U) || (scale == 0U))
  {
    return;
  }

  abs_value = (value < 0) ? -(int64_t) value : (int64_t) value;
  int_part = (long) (abs_value / (int64_t) scale);
  frac_part = (long) (abs_value % (int64_t) scale);
  sign = (value < 0) ? "-" : "";

  if (frac_digits == 2U)
  {
    (void) snprintf(buffer, buffer_len, "%s%ld.%02ld", sign, int_part, frac_part);
  }
  else if (frac_digits == 3U)
  {
    (void) snprintf(buffer, buffer_len, "%s%ld.%03ld", sign, int_part, frac_part);
  }
  else if (frac_digits == 7U)
  {
    (void) snprintf(buffer, buffer_len, "%s%ld.%07ld", sign, int_part, frac_part);
  }
  else
  {
    (void) snprintf(buffer, buffer_len, "%s%ld.%04ld", sign, int_part, frac_part);
  }
}

static void csv_log_imu_sample(const struct bmi2_sens_data *imu_sample,
                               const gps_status_t *gps_sample,
                               uint32_t log_tick)
{
  SystemTime_t time_snapshot;
  char system_time_buffer[20];
  char acc_x_buffer[20];
  char acc_y_buffer[20];
  char acc_z_buffer[20];
  char gyr_x_buffer[20];
  char gyr_y_buffer[20];
  char gyr_z_buffer[20];
  const char *gps_utc;
  int32_t acc_x_x10000;
  int32_t acc_y_x10000;
  int32_t acc_z_x10000;
  int32_t gyr_x_x1000;
  int32_t gyr_y_x1000;
  int32_t gyr_z_x1000;

  if ((data_log_ready == 0U) || (imu_sample == NULL) || (gps_sample == NULL))
  {
    return;
  }

  system_time_advance_to_tick(log_tick);

  __disable_irq();
  time_snapshot = system_time;
  __enable_irq();

  system_time_format(&time_snapshot, system_time_buffer, sizeof(system_time_buffer));
  gps_utc = (gps_sample->valid != 0U) ? gps_sample->utc : "";

  /* Fixed-point conversion avoids %f in FatFs f_printf, which is not supported. */
  acc_x_x10000 = div_round_s32((int64_t) imu_sample->acc.x * 10000LL, 2048);
  acc_y_x10000 = div_round_s32((int64_t) imu_sample->acc.y * 10000LL, 2048);
  acc_z_x10000 = div_round_s32((int64_t) imu_sample->acc.z * 10000LL, 2048);
  gyr_x_x1000 = div_round_s32((int64_t) imu_sample->gyr.x * 62500LL, 1024);
  gyr_y_x1000 = div_round_s32((int64_t) imu_sample->gyr.y * 62500LL, 1024);
  gyr_z_x1000 = div_round_s32((int64_t) imu_sample->gyr.z * 62500LL, 1024);

  format_fixed_decimal(acc_x_buffer, sizeof(acc_x_buffer), acc_x_x10000, 10000U, 4U);
  format_fixed_decimal(acc_y_buffer, sizeof(acc_y_buffer), acc_y_x10000, 10000U, 4U);
  format_fixed_decimal(acc_z_buffer, sizeof(acc_z_buffer), acc_z_x10000, 10000U, 4U);
  format_fixed_decimal(gyr_x_buffer, sizeof(gyr_x_buffer), gyr_x_x1000, 1000U, 3U);
  format_fixed_decimal(gyr_y_buffer, sizeof(gyr_y_buffer), gyr_y_x1000, 1000U, 3U);
  format_fixed_decimal(gyr_z_buffer, sizeof(gyr_z_buffer), gyr_z_x1000, 1000U, 3U);

  (void) f_printf(&imu_log_file,
                  "%s,%s,%s,%s,%s,%s,%s,%s\r\n",
                  system_time_buffer,
                  gps_utc,
                  acc_x_buffer,
                  acc_y_buffer,
                  acc_z_buffer,
                  gyr_x_buffer,
                  gyr_y_buffer,
                  gyr_z_buffer);
}

static void csv_log_baro_sample(const struct bmp3_data *baro_sample,
                                const gps_status_t *gps_sample,
                                uint32_t log_tick)
{
  SystemTime_t time_snapshot;
  char system_time_buffer[20];
  char temperature_buffer[20];
  char pressure_buffer[20];
  const char *gps_utc;
  int32_t temperature_x100;
  int32_t pressure_pa;

  if ((data_log_ready == 0U) || (baro_sample == NULL) || (gps_sample == NULL))
  {
    return;
  }

  system_time_advance_to_tick(log_tick);

  __disable_irq();
  time_snapshot = system_time;
  __enable_irq();

  system_time_format(&time_snapshot, system_time_buffer, sizeof(system_time_buffer));
  gps_utc = (gps_sample->valid != 0U) ? gps_sample->utc : "";
  temperature_x100 = div_round_s32((int64_t) (baro_sample->temperature * 1000.0), 10);
  pressure_pa = (int32_t) (baro_sample->pressure);

  format_fixed_decimal(temperature_buffer, sizeof(temperature_buffer), temperature_x100, 100U, 2U);
  format_fixed_decimal(pressure_buffer, sizeof(pressure_buffer), pressure_pa, 1000U, 3U);

  (void) f_printf(&baro_log_file,
                  "%s,%s,%s,%s\r\n",
                  system_time_buffer,
                  gps_utc,
                  temperature_buffer,
                  pressure_buffer);
}

static void csv_log_gps_sample(const gps_status_t *gps_sample,
                               uint32_t log_tick)
{
  SystemTime_t time_snapshot;
  char system_time_buffer[20];
  char lat_buffer[24];
  char lon_buffer[24];
  char alt_buffer[20];
  int32_t lat_x1e7;
  int32_t lon_x1e7;
  int32_t alt_x100;

  if ((data_log_ready == 0U) || (gps_sample == NULL))
  {
    return;
  }

  system_time_advance_to_tick(log_tick);

  __disable_irq();
  time_snapshot = system_time;
  __enable_irq();

  system_time_format(&time_snapshot, system_time_buffer, sizeof(system_time_buffer));

  if (gps_sample->valid != 0U)
  {
    lat_x1e7 = div_round_s32((int64_t) (gps_sample->latitude_deg * 100000000.0), 10);
    lon_x1e7 = div_round_s32((int64_t) (gps_sample->longitude_deg * 100000000.0), 10);
    alt_x100 = div_round_s32((int64_t) (gps_sample->altitude_m * 1000.0), 10);

    format_fixed_decimal(lat_buffer, sizeof(lat_buffer), lat_x1e7, 10000000U, 7U);
    format_fixed_decimal(lon_buffer, sizeof(lon_buffer), lon_x1e7, 10000000U, 7U);
    format_fixed_decimal(alt_buffer, sizeof(alt_buffer), alt_x100, 100U, 2U);

    (void) f_printf(&gps_log_file,
                    "%s,%s,%s,%s,%s,%u\r\n",
                    system_time_buffer,
                    gps_sample->utc,
                    lat_buffer,
                    lon_buffer,
                    alt_buffer,
                    (unsigned int) gps_sample->satellites);
  }
  else
  {
    (void) f_printf(&gps_log_file,
                    "%s,,,,,%u\r\n",
                    system_time_buffer,
                    0U);
  }
}

/* Start USART1 receive-to-idle on the circular GPS RX buffer. */
static void gps_uart_start_reception(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_rx_dma_buffer, GPS_RX_DMA_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    /* Do not kill the whole firmware if GPS RX bring-up fails.
     * Keep the rest of the application alive so we can debug the issue from RTT.
     */
    SEGGER_RTT_printf("GPSDBG|UART start failed: %d\r\n", (int) status);
    gps_uart_started_ok = 0U;
    return;
  }

  gps_uart_started_ok = 1U;
  SEGGER_RTT_printf("GPSDBG|UART start ok baud=%lu\r\n", (unsigned long) huart1.Init.BaudRate);

  /* Half-transfer interrupts are not needed for NMEA line handling. */
  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }

  gps_rx_old_pos = 0U;
  gps_line_length = 0U;
  gps_uart_rx_event_count = 0U;
  gps_uart_rx_byte_count = 0U;
  gps_uart_last_chunk_size = 0U;
  gps_uart_error_code = 0U;
  gps_uart_last_line_length = 0U;
  gps_uart_last_line[0] = '\0';
  gps_ubx_stream_length = 0U;
}

/* Handle one new slice from the circular DMA buffer. */
static void gps_uart_process_dma_chunk(uint16_t new_pos)
{
  uint16_t pos;
  uint16_t chunk_size;

  if (new_pos > GPS_RX_DMA_BUFFER_SIZE)
  {
    new_pos = GPS_RX_DMA_BUFFER_SIZE;
  }

  pos = gps_rx_old_pos;

  if (new_pos == pos)
  {
    return;
  }

  chunk_size = (new_pos > pos) ? (uint16_t) (new_pos - pos) : (uint16_t) ((GPS_RX_DMA_BUFFER_SIZE - pos) + new_pos);
  gps_uart_last_chunk_size = chunk_size;
  gps_uart_rx_event_count++;
  gps_uart_rx_byte_count += chunk_size;

#if GPS_EXTENSIVE_DEBUG
  SEGGER_RTT_printf("GPSDBG|chunk new=%u old=%u size=%u total=%lu\r\n",
                    (unsigned int) new_pos,
                    (unsigned int) pos,
                    (unsigned int) chunk_size,
                    (unsigned long) gps_uart_rx_byte_count);
#endif

  if (new_pos > pos)
  {
    gps_uart_dump_chunk(&gps_rx_dma_buffer[pos], (uint16_t) (new_pos - pos));
    while (pos < new_pos)
    {
      gps_uart_process_byte(gps_rx_dma_buffer[pos]);
      pos++;
    }
  }
  else
  {
    gps_uart_dump_chunk(&gps_rx_dma_buffer[pos], (uint16_t) (GPS_RX_DMA_BUFFER_SIZE - pos));
    while (pos < GPS_RX_DMA_BUFFER_SIZE)
    {
      gps_uart_process_byte(gps_rx_dma_buffer[pos]);
      pos++;
    }

    pos = 0U;
    gps_uart_dump_chunk(&gps_rx_dma_buffer[pos], new_pos);
    while (pos < new_pos)
    {
      gps_uart_process_byte(gps_rx_dma_buffer[pos]);
      pos++;
    }
  }

  gps_rx_old_pos = new_pos;
}

/* Assemble NMEA lines from raw bytes and parse complete GGA sentences. */
static void gps_uart_process_byte(uint8_t byte)
{
  char c = (char) byte;

  /* Feed all incoming bytes into official u-blox UBX decoder. */
  gps_ubx_feed_byte(byte);

  if (c == '\r')
  {
    return;
  }

  if (c == '\n')
  {
    if (gps_line_length > 0U)
    {
      gps_line_buffer[gps_line_length] = '\0';

      gps_uart_last_line_length = gps_line_length;
      memcpy(gps_uart_last_line, gps_line_buffer, gps_line_length + 1U);

#if GPS_EXTENSIVE_DEBUG
      SEGGER_RTT_printf("GPSLINE|len=%u|%s\r\n",
                        (unsigned int) gps_line_length,
                        gps_line_buffer);
#endif

      (void) gps_parse_gga_sentence(gps_line_buffer);
    }

    gps_line_length = 0U;
    return;
  }

  if (gps_line_length < (GPS_NMEA_LINE_BUFFER_SIZE - 1U))
  {
    gps_line_buffer[gps_line_length] = c;
    gps_line_length++;
  }
  else
  {
    /* Drop the partial sentence if it overflows the line buffer. */
    gps_line_length = 0U;
  }
}

/* Feed one byte into UBX decode stream and parse complete UBX messages. */
static void gps_ubx_feed_byte(uint8_t byte)
{
  int32_t body_len;
  int32_t message_class = 0;
  int32_t message_id = 0;
  const char *p_out = NULL;
  size_t consumed;

  if (gps_ubx_stream_length < sizeof(gps_ubx_stream_buffer))
  {
    gps_ubx_stream_buffer[gps_ubx_stream_length] = byte;
    gps_ubx_stream_length++;
  }
  else
  {
    /* Keep latest bytes if the stream buffer overflows. */
    memmove(gps_ubx_stream_buffer,
            gps_ubx_stream_buffer + (sizeof(gps_ubx_stream_buffer) / 2U),
            sizeof(gps_ubx_stream_buffer) / 2U);
    gps_ubx_stream_length = sizeof(gps_ubx_stream_buffer) / 2U;
    gps_ubx_stream_buffer[gps_ubx_stream_length] = byte;
    gps_ubx_stream_length++;
  }

  while (gps_ubx_stream_length > 0U)
  {
    body_len = uUbxProtocolDecode((const char *) gps_ubx_stream_buffer,
                                  gps_ubx_stream_length,
                                  &message_class,
                                  &message_id,
                                  gps_ubx_body_buffer,
                                  sizeof(gps_ubx_body_buffer),
                                  &p_out);

    if (p_out == NULL)
    {
      break;
    }

    consumed = (size_t) (p_out - (const char *) gps_ubx_stream_buffer);

    if (body_len == (int32_t) U_ERROR_COMMON_TIMEOUT)
    {
      /* Keep partial UBX message until more bytes arrive. */
      break;
    }

    if (body_len >= 0)
    {
#if GPS_EXTENSIVE_DEBUG
      SEGGER_RTT_printf("GPSUBX|class=0x%02X id=0x%02X body=%ld\r\n",
                        (unsigned int) message_class,
                        (unsigned int) message_id,
                        (long) body_len);
#endif
      if ((message_class == 0x01) && (message_id == 0x07))
      {
        gps_ubx_parse_nav_pvt(gps_ubx_body_buffer, body_len);
      }
    }

    if ((consumed == 0U) || (consumed > gps_ubx_stream_length))
    {
      /* Safety net: if decoder didn't advance, drop one byte and retry later. */
      memmove(gps_ubx_stream_buffer, gps_ubx_stream_buffer + 1, gps_ubx_stream_length - 1U);
      gps_ubx_stream_length--;
    }
    else
    {
      memmove(gps_ubx_stream_buffer,
              gps_ubx_stream_buffer + consumed,
              gps_ubx_stream_length - consumed);
      gps_ubx_stream_length -= consumed;
    }
  }
}

/* Parse UBX-NAV-PVT body fields (UTC, fix type, satellite count). */
static void gps_ubx_parse_nav_pvt(const char *body, int32_t body_len)
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t fix_type;
  uint8_t num_sv;
  int32_t lon_x1e7;
  int32_t lat_x1e7;
  int32_t height_mm;
  int32_t hmsl_mm;
  uint8_t fix_quality;

  if ((body == NULL) || (body_len < 40))
  {
    return;
  }

  year = uUbxProtocolUint16Decode(body + 4);
  month = (uint8_t) body[6];
  day = (uint8_t) body[7];
  hour = (uint8_t) body[8];
  minute = (uint8_t) body[9];
  second = (uint8_t) body[10];
  fix_type = (uint8_t) body[20];
  num_sv = (uint8_t) body[23];
  lon_x1e7 = (int32_t) uUbxProtocolUint32Decode(body + 24);
  lat_x1e7 = (int32_t) uUbxProtocolUint32Decode(body + 28);
  height_mm = (int32_t) uUbxProtocolUint32Decode(body + 32);
  hmsl_mm = (int32_t) uUbxProtocolUint32Decode(body + 36);

  (void) day;
  (void) month;
  (void) year;
  (void) height_mm;

  (void) snprintf(gps_status.utc,
                  sizeof(gps_status.utc),
                  "%02u:%02u:%02u",
                  (unsigned int) hour,
                  (unsigned int) minute,
                  (unsigned int) second);
  fix_quality = (fix_type >= 2U) ? 1U : 0U;
  gps_status.fix_quality = fix_quality;
  gps_status.satellites = num_sv;
  gps_status.longitude_deg = (double) lon_x1e7 / 10000000.0;
  gps_status.latitude_deg = (double) lat_x1e7 / 10000000.0;
  gps_status.altitude_m = (double) hmsl_mm / 1000.0;
  gps_status.valid = (fix_quality != 0U) ? 1U : 0U;
  gps_status_dirty = 1U;

  if (fix_quality != 0U)
  {
    system_time_set_from_utc(gps_status.utc);
  }

#if GPS_EXTENSIVE_DEBUG
  SEGGER_RTT_printf("GPSPVT|utc=%s fix_type=%u sats=%u\r\n",
                    gps_status.utc,
                    (unsigned int) fix_type,
                    (unsigned int) gps_status.satellites);
#endif
}

/* Poll DMA write position as a fallback when RxEvent callback is not triggered. */
static void gps_uart_poll_dma(void)
{
  uint16_t new_pos;

  if ((gps_uart_started_ok == 0U) || (huart1.hdmarx == NULL))
  {
    return;
  }

  new_pos = (uint16_t) (GPS_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
  gps_uart_process_dma_chunk(new_pos);
}

/* Parse and publish one GGA sentence.
 * The parser is intentionally small: it only extracts UTC time, fix quality,
 * and satellite count because that is enough to validate GPS lock and antenna health.
 */
static uint8_t gps_parse_gga_sentence(const char *line)
{
  const char *field_start;
  const char *field_end;
  uint8_t fix_quality;
  uint8_t satellites;

  if ((line == NULL) || (line[0] != '$') || (strncmp(&line[3], "GGA,", 4U) != 0))
  {
    return 0U;
  }

  if (!gps_validate_checksum(line))
  {
    return 0U;
  }

  /* Field 1: UTC time (hhmmss.sss). */
  field_start = &line[7];
  field_end = strchr(field_start, ',');
  if (field_end == NULL)
  {
    return 0U;
  }

  if (!gps_format_utc_field(field_start,
                            (size_t) (field_end - field_start),
                            gps_status.utc,
                            sizeof(gps_status.utc)))
  {
    return 0U;
  }

  /* Field 2: latitude and hemisphere. */
  field_start = field_end + 1;
  field_end = strchr(field_start, ',');
  if (field_end == NULL)
  {
    return 0U;
  }
  if ((field_end > field_start) && gps_parse_nmea_coordinate(field_start, field_end, field_end[1], &gps_status.latitude_deg))
  {
    /* Latitude stored in gps_status. */
  }
  else
  {
    gps_status.latitude_deg = 0.0;
  }

  /* Field 4: longitude and hemisphere. */
  field_start = gps_skip_field(field_end + 1);
  if (field_start == NULL)
  {
    return 0U;
  }

  field_end = strchr(field_start, ',');
  if (field_end == NULL)
  {
    return 0U;
  }
  if ((field_end > field_start) && gps_parse_nmea_coordinate(field_start, field_end, field_end[1], &gps_status.longitude_deg))
  {
    /* Longitude stored in gps_status. */
  }
  else
  {
    gps_status.longitude_deg = 0.0;
  }

  /* Field 6: fix quality. */
  field_start = gps_skip_field(field_end + 1);
  if (field_start == NULL)
  {
    return 0U;
  }
  field_end = strchr(field_start, ',');
  if (field_end == NULL)
  {
    field_end = strchr(field_start, '*');
  }
  if ((field_end == NULL) || !gps_parse_uint8_field(field_start, field_end, &fix_quality))
  {
    return 0U;
  }

  /* Field 7: number of satellites. */
  field_start = gps_skip_field(field_start);
  if (field_start == NULL)
  {
    return 0U;
  }

  field_end = strchr(field_start, ',');
  if (field_end == NULL)
  {
    field_end = strchr(field_start, '*');
  }
  if ((field_end == NULL) || !gps_parse_uint8_field(field_start, field_end, &satellites))
  {
    return 0U;
  }

  /* Field 9: altitude above mean sea level. */
  gps_status.altitude_m = 0.0;
  field_start = gps_skip_field(field_start);
  if (field_start != NULL)
  {
    field_start = gps_skip_field(field_start);
    if (field_start != NULL)
    {
      field_end = strchr(field_start, ',');
      if (field_end == NULL)
      {
        field_end = strchr(field_start, '*');
      }
      if ((field_end != NULL) && (field_end > field_start))
      {
        char altitude_buffer[24];
        size_t length = (size_t) (field_end - field_start);
        char *parse_end;

        if (length < sizeof(altitude_buffer))
        {
          memcpy(altitude_buffer, field_start, length);
          altitude_buffer[length] = '\0';
          gps_status.altitude_m = strtod(altitude_buffer, &parse_end);
          if ((parse_end == altitude_buffer) || (*parse_end != '\0'))
          {
            gps_status.altitude_m = 0.0;
          }
        }
        else
        {
          gps_status.altitude_m = 0.0;
        }
      }
      else
      {
        gps_status.altitude_m = 0.0;
      }
    }
  }

  gps_status.fix_quality = fix_quality;
  gps_status.satellites = satellites;
  gps_status.valid = (fix_quality != 0U) ? 1U : 0U;
  gps_status_dirty = 1U;

  if (fix_quality != 0U)
  {
    system_time_set_from_utc(gps_status.utc);
  }

#if GPS_EXTENSIVE_DEBUG
  SEGGER_RTT_printf("GPSGGA|utc=%s fix=%u sats=%u\r\n",
                    gps_status.utc,
                    (unsigned int) gps_status.fix_quality,
                    (unsigned int) gps_status.satellites);
#endif

  return 1U;
}

/* Temporary bring-up diagnostics to answer: are bytes arriving on USART1? */
static void gps_uart_report_debug(void)
{
  uint32_t rx_event_count;
  uint32_t rx_byte_count;
  uint32_t error_code;
  uint32_t cndtr = 0U;
  uint32_t usart_isr = 0U;
  uint16_t last_chunk_size;
  uint16_t last_line_length;

  __disable_irq();
  rx_event_count = gps_uart_rx_event_count;
  rx_byte_count = gps_uart_rx_byte_count;
  last_chunk_size = gps_uart_last_chunk_size;
  last_line_length = gps_uart_last_line_length;
  error_code = gps_uart_error_code;
  __enable_irq();

  if (huart1.hdmarx != NULL)
  {
    cndtr = huart1.hdmarx->Instance->CNDTR;
  }
  usart_isr = huart1.Instance->ISR;

  if (gps_uart_started_ok == 0U)
  {
    SEGGER_RTT_printf("GPSDBG|rx not started\r\n");
    return;
  }

  if (rx_event_count == 0U)
  {
    SEGGER_RTT_printf("GPSDBG|no_rx_events bytes=%lu cndtr=%lu isr=0x%08lX err=0x%08lX last_line=%u\r\n",
                      (unsigned long) rx_byte_count,
                      (unsigned long) cndtr,
                      (unsigned long) usart_isr,
                      (unsigned long) error_code,
                      (unsigned int) last_line_length);
    return;
  }

  SEGGER_RTT_printf("GPSDBG|rx_events=%lu bytes=%lu last_chunk=%u cndtr=%lu isr=0x%08lX err=0x%08lX last_line=%u\r\n",
                    (unsigned long) rx_event_count,
                    (unsigned long) rx_byte_count,
                    (unsigned int) last_chunk_size,
                    (unsigned long) cndtr,
                    (unsigned long) usart_isr,
                    (unsigned long) error_code);

#if GPS_EXTENSIVE_DEBUG
  if (gps_uart_last_line_length > 0U)
  {
    SEGGER_RTT_printf("GPSLAST|%s\r\n", gps_uart_last_line);
  }
#endif
}

/* Dump up to the first 32 bytes of a DMA chunk as hex so we can see raw UART data. */
static void gps_uart_dump_chunk(const uint8_t *data, uint16_t len)
{
#if GPS_EXTENSIVE_DEBUG
  uint16_t i;
  uint16_t limit;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  limit = (len > 32U) ? 32U : len;
  SEGGER_RTT_printf("GPSHEX|len=%u|", (unsigned int) len);
  for (i = 0U; i < limit; i++)
  {
    SEGGER_RTT_printf("%02X ", (unsigned int) data[i]);
  }
  if (len > limit)
  {
    SEGGER_RTT_printf("...");
  }
  SEGGER_RTT_printf("\r\n");
#else
  (void) data;
  (void) len;
#endif
}

/* Skip to the character immediately after the next comma-delimited field. */
static const char *gps_skip_field(const char *field)
{
  const char *comma;

  if (field == NULL)
  {
    return NULL;
  }

  comma = strchr(field, ',');
  if (comma == NULL)
  {
    return NULL;
  }

  return comma + 1;
}

/* Parse an unsigned 8-bit integer from a bounded NMEA field. */
static uint8_t gps_parse_uint8_field(const char *start, const char *end, uint8_t *value)
{
  uint32_t result = 0U;
  const char *p;

  if ((start == NULL) || (end == NULL) || (value == NULL) || (start >= end))
  {
    return 0U;
  }

  for (p = start; p < end; p++)
  {
    if ((*p < '0') || (*p > '9'))
    {
      return 0U;
    }

    result = (result * 10U) + (uint32_t) (*p - '0');
    if (result > 255U)
    {
      return 0U;
    }
  }

  *value = (uint8_t) result;
  return 1U;
}

/* Parse a ddmm.mmmm or dddmm.mmmm NMEA coordinate into signed degrees. */
static uint8_t gps_parse_nmea_coordinate(const char *start, const char *end, char hemisphere, double *value)
{
  char coordinate_buffer[24];
  size_t length;
  char *parse_end;
  double raw_value;
  int degrees;
  double minutes;

  if ((start == NULL) || (end == NULL) || (value == NULL) || (start >= end))
  {
    return 0U;
  }

  length = (size_t) (end - start);
  if ((length == 0U) || (length >= sizeof(coordinate_buffer)))
  {
    return 0U;
  }

  memcpy(coordinate_buffer, start, length);
  coordinate_buffer[length] = '\0';

  raw_value = strtod(coordinate_buffer, &parse_end);
  if ((parse_end == coordinate_buffer) || (*parse_end != '\0'))
  {
    return 0U;
  }

  degrees = (int) (raw_value / 100.0);
  minutes = raw_value - ((double) degrees * 100.0);
  *value = (double) degrees + (minutes / 60.0);

  if ((hemisphere == 'S') || (hemisphere == 'W'))
  {
    *value = -(*value);
  }

  return 1U;
}

/* Format NMEA UTC hhmmss.sss into a readable HH:MM:SS[.ss] string. */
static uint8_t gps_format_utc_field(const char *start, size_t length, char *out, size_t out_len)
{
  size_t i;
  size_t written;
  size_t frac_len;
  size_t dot_index;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  if ((start == NULL) || (out == NULL) || (out_len < 9U) || (length < 6U))
  {
    return 0U;
  }

  for (i = 0U; i < 6U; i++)
  {
    if ((start[i] < '0') || (start[i] > '9'))
    {
      return 0U;
    }
  }

  hour = (uint8_t) ((start[0] - '0') * 10U + (start[1] - '0'));
  minute = (uint8_t) ((start[2] - '0') * 10U + (start[3] - '0'));
  second = (uint8_t) ((start[4] - '0') * 10U + (start[5] - '0'));

  written = (size_t) snprintf(out, out_len, "%02u:%02u:%02u",
                              (unsigned int) hour,
                              (unsigned int) minute,
                              (unsigned int) second);
  if ((written == 0U) || (written >= out_len))
  {
    return 0U;
  }

  dot_index = length;
  for (i = 0U; i < length; i++)
  {
    if (start[i] == '.')
    {
      dot_index = i;
      break;
    }
  }

  if (dot_index < length)
  {
    frac_len = length - dot_index - 1U;
    if (frac_len > 2U)
    {
      frac_len = 2U;
    }

    if ((frac_len > 0U) && ((written + 1U + frac_len) < out_len))
    {
      out[written] = '.';
      written++;
      for (i = 0U; i < frac_len; i++)
      {
        out[written] = start[dot_index + 1U + i];
        written++;
      }

      out[written] = '\0';
    }
  }

  return 1U;
}

/* Validate the NMEA checksum if the sentence carries one. */
static uint8_t gps_validate_checksum(const char *line)
{
  const char *asterisk;
  const char *p;
  uint8_t checksum;
  int8_t hi;
  int8_t lo;

  if (line == NULL)
  {
    return 0U;
  }

  asterisk = strchr(line, '*');
  if (asterisk == NULL)
  {
    return 1U;
  }

  checksum = 0U;
  for (p = &line[1]; (p < asterisk) && (*p != '\0'); p++)
  {
    checksum ^= (uint8_t) *p;
  }

  if ((asterisk[1] == '\0') || (asterisk[2] == '\0'))
  {
    return 0U;
  }

  hi = gps_hex_to_nibble(asterisk[1]);
  lo = gps_hex_to_nibble(asterisk[2]);
  if ((hi < 0) || (lo < 0))
  {
    return 0U;
  }

  return ((uint8_t) ((hi << 4) | lo) == checksum) ? 1U : 0U;
}

/* Convert a hex character into a 0-15 nibble. */
static int8_t gps_hex_to_nibble(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (int8_t) (c - '0');
  }

  if ((c >= 'a') && (c <= 'f'))
  {
    return (int8_t) (10 + (c - 'a'));
  }

  if ((c >= 'A') && (c <= 'F'))
  {
    return (int8_t) (10 + (c - 'A'));
  }

  return (int8_t) -1;
}

/* Emit the latest parsed GPS status as a compact RTT line.
 * This stays separate from the IMU CSV stream by using a distinct prefix.
 */
static void gps_emit_status(void)
{
  gps_status_t snapshot;

  if (gps_status_dirty == 0U)
  {
    return;
  }

  __disable_irq();
  snapshot = gps_status;
  gps_status_dirty = 0U;
  __enable_irq();

  SEGGER_RTT_printf("GPS|UTC=%s Fix=%u Sats=%u\r\n",
                    snapshot.utc,
                    (unsigned int) snapshot.fix_quality,
                    (unsigned int) snapshot.satellites);
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
                            I2C_XFER_TIMEOUT_MS);

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
                             I2C_XFER_TIMEOUT_MS);

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
                            I2C_XFER_TIMEOUT_MS);

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
                             I2C_XFER_TIMEOUT_MS);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  /* STEP A: Initialize SEGGER RTT for debug output over SWD/ST-Link.
   * This must happen before any printf() calls.
   * RTT uses the debug connection (no UART needed for debug output).
   */
  SEGGER_RTT_Init();

  /* Release GPS reset explicitly.
   * GPS_RESET is active-low on this hardware, so the module will stay off if this pin remains low.
   */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  /* Enable the high-resolution DWT cycle counter for PPS capture. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Start GPS receive-to-idle on USART1 DMA before entering the main loop. */
  gps_uart_start_reception();

  /* STEP D: first visible debug print. If you see this in RTT, CPU + debug link are alive. */
  printf("Booting...\r\n");

  /* STEP E: initialize sensor stack (callbacks + Bosch init + accel config). */
  /* Stop here if sensor init fails so failure is obvious during bring-up. */
  /* [OURS] We choose when init happens and what to do on failure. */
  if (bmi270_basic_init() != BMI2_OK)
  {
    SEGGER_RTT_printf("BMI270 unavailable, continuing without IMU\r\n");
  }

  /* STEP E2: initialize BMP390 (Bosch BMP3 API) on I2C2 for pressure + temperature. */
  if (bmp390_basic_init() != BMP3_OK)
  {
    SEGGER_RTT_printf("BMP390 unavailable, continuing without barometer\r\n");
  }

  /* STEP F: set known initial state before entering endless loop.
   * Setting baseline ticks now avoids an immediate first-loop trigger.
   */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  last_blink_tick = HAL_GetTick();
  last_imu_tick = HAL_GetTick();
  last_baro_tick = HAL_GetTick();
  last_gps_log_tick = HAL_GetTick();
  last_imu_stats_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* TASK 0: consume PPS events in the main loop and discipline the system clock. */
    if (gps_pps_sync_ready != 0U)
    {
      uint32_t pps_tick;

      __disable_irq();
      pps_tick = gps_pps_tick_stamp;
      gps_pps_sync_ready = 0U;
      __enable_irq();

      system_time_handle_pps(pps_tick);
    }

    /* TASK 1: process button events captured by ISR. */
    /* Button ISR sets button_event; main loop handles behavior change. */
    /* [OURS] This whole control logic is yours to understand deeply. */
    if (button_event)
    {
      /* Consume the event once, then clear it.
       * Event is set in EXTI callback after debounce.
       */
      button_event = 0;
      if (logging_active == 0U)
      {
        if (data_log_start() == FR_OK)
        {
          logging_active = 1U;
          last_blink_tick = HAL_GetTick();
        }
      }
      else
      {
        data_log_stop();
        logging_active = 0U;
      }
    }

    /* TASK 2: blink LED only while logging is active. */
    /* Non-blocking blink task: only active while SD logging is enabled. */
    /* [OURS] Timing/state pattern is yours. */
    if (logging_active != 0U)
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
    else
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
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
      uint32_t log_tick;
      gps_status_t gps_snapshot;

      /* Same timing pattern as blink task: do work only when period elapsed. */
      log_tick = HAL_GetTick();
      last_imu_tick = log_tick;
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

      gps_copy_status_snapshot(&gps_snapshot);
      csv_log_imu_sample(&bmi_sens_data, &gps_snapshot, log_tick);
    }

    /* TASK 4: periodically read BMP390 pressure+temperature. */
    if ((HAL_GetTick() - last_baro_tick) >= BARO_VISUAL_REFRESH_MS)
    {
      int8_t rslt;
      uint32_t log_tick;
      gps_status_t gps_snapshot;

      log_tick = HAL_GetTick();
      last_baro_tick = log_tick;
      rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp_data, &bmp_dev);
      if (rslt == BMP3_OK)
      {
        int32_t pressure_pa = (int32_t) (bmp_data.pressure);
        int32_t temp_c_x100 = (int32_t) (bmp_data.temperature * 100.0);

        baro_read_ok_count++;
        baro_last_pressure_pa = pressure_pa;
        baro_last_temp_c_x100 = temp_c_x100;
        baro_last_valid = 1U;
        gps_copy_status_snapshot(&gps_snapshot);
        csv_log_baro_sample(&bmp_data, &gps_snapshot, log_tick);
      }
      else
      {
        baro_read_err_count++;
        (void) rslt;
      }
    }

    /* TASK 5: log GPS data to file at 1 Hz, independent of IMU/baro rates. */
    if ((HAL_GetTick() - last_gps_log_tick) >= GPS_LOG_REFRESH_MS)
    {
      uint32_t log_tick;
      gps_status_t gps_snapshot;

      log_tick = HAL_GetTick();
      last_gps_log_tick = log_tick;
      gps_copy_status_snapshot(&gps_snapshot);
      csv_log_gps_sample(&gps_snapshot, log_tick);
    }

    /* TASK 5b: perform periodic SD sync in one place to avoid clustered blocking writes. */
    if ((data_log_ready != 0U) && ((HAL_GetTick() - data_log_last_sync_tick) >= DATA_LOG_SYNC_MS))
    {
      data_log_last_sync_tick = HAL_GetTick();
      (void) f_sync(&imu_log_file);
      (void) f_sync(&baro_log_file);
      (void) f_sync(&gps_log_file);
    }

    /* TASK 6: print low-rate sensor health summary once per second. */
    if ((HAL_GetTick() - last_imu_stats_tick) >= 1000U)
    {
      last_imu_stats_tick = HAL_GetTick();
      printf("\r\nHEALTH|IMU ok:%lu err:%lu | BARO ok:%lu err:%lu | GPS rx_events:%lu bytes:%lu err:0x%08lX\r\n",
             (unsigned long) imu_read_ok_count,
             (unsigned long) imu_read_err_count,
             (unsigned long) baro_read_ok_count,
             (unsigned long) baro_read_err_count,
             (unsigned long) gps_uart_rx_event_count,
             (unsigned long) gps_uart_rx_byte_count,
             (unsigned long) gps_uart_error_code);

      if (baro_last_valid)
      {
        printf("BARO|P=%ldPa T=%ld.%02ldC\r\n",
               (long) baro_last_pressure_pa,
               (long) (baro_last_temp_c_x100 / 100),
               (long) ((baro_last_temp_c_x100 < 0) ? -(baro_last_temp_c_x100 % 100) : (baro_last_temp_c_x100 % 100)));
      }

      if (gps_uart_last_line_length > 0U)
      {
        SEGGER_RTT_printf("GPSRAW1HZ|%s\r\n", gps_uart_last_line);
      }

      /* Keep UART/GPS transport debug to 1 Hz so lock progress is visible without RTT flooding. */
      gps_uart_report_debug();
    }

    /* TASK 7: publish the latest GPS status line when a new GGA sentence has been parsed. */
    gps_uart_poll_dma();
    gps_emit_status();
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPS_EINT_Pin|GPS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_PPS_Pin */
  GPIO_InitStruct.Pin = GPS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_PPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_EINT_Pin GPS_RST_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = GPS_EINT_Pin|GPS_RST_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Receive-to-idle callback runs from the UART interrupt path.
 * We only move bytes into a line buffer and parse the sentence; the RTT print happens in the main loop.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    gps_uart_process_dma_chunk(Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    gps_uart_error_code = huart->ErrorCode;
    SEGGER_RTT_printf("GPSDBG|UART error: 0x%08lX\r\n", (unsigned long) huart->ErrorCode);
    gps_uart_start_reception();
  }
}

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
  else if (GPIO_Pin == GPS_PPS_Pin)
  {
    /* Capture a high-resolution synchronization point for GPS/IMU drift correction. */
    gps_pps_cycle_stamp = DWT->CYCCNT;
    gps_pps_tick_stamp = HAL_GetTick();
    gps_pps_sync_ready = 1U;
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
