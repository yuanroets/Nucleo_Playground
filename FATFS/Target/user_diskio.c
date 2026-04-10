/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_diskio.c
  * @brief   FatFS disk I/O glue for an SD card on SPI2.
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SD_SPI_TIMEOUT_MS 100U
#define SD_READY_TIMEOUT_MS 500U
#define SD_INIT_TIMEOUT_MS 1000U
#define SD_READ_RETRY_COUNT 2U

/* SD driver diagnostic stages for RTT troubleshooting. */
#define SD_DIAG_STAGE_IDLE                  0x00U
#define SD_DIAG_STAGE_INIT_START            0x10U
#define SD_DIAG_STAGE_INIT_CMD0_FAIL        0x11U
#define SD_DIAG_STAGE_INIT_CMD8_FAIL        0x12U
#define SD_DIAG_STAGE_INIT_ACMD41_TIMEOUT   0x13U
#define SD_DIAG_STAGE_INIT_CMD58_FAIL       0x14U
#define SD_DIAG_STAGE_INIT_CMD16_FAIL       0x15U
#define SD_DIAG_STAGE_INIT_OK               0x1FU
#define SD_DIAG_STAGE_READ_CMD_FAIL         0x20U
#define SD_DIAG_STAGE_READ_TOKEN_FAIL       0x21U
#define SD_DIAG_STAGE_WRITE_CMD_FAIL        0x30U
#define SD_DIAG_STAGE_WRITE_TOKEN_FAIL      0x31U
#define SD_DIAG_STAGE_IOCTL_FAIL            0x40U

/* Card type flags (CardType). */
#define CT_MMC    0x01U
#define CT_SD1    0x02U
#define CT_SD2    0x04U
#define CT_BLOCK  0x08U

/* Command packet prefixes. */
#define CMD0    (0U)
#define CMD1    (1U)
#define CMD8    (8U)
#define CMD9    (9U)
#define CMD10   (10U)
#define CMD12   (12U)
#define CMD16   (16U)
#define CMD17   (17U)
#define CMD18   (18U)
#define CMD23   (23U)
#define CMD24   (24U)
#define CMD25   (25U)
#define CMD55   (55U)
#define CMD58   (58U)
#define ACMD13  (0x80U + 13U)
#define ACMD23  (0x80U + 23U)
#define ACMD41  (0x80U + 41U)

extern SPI_HandleTypeDef hspi2;

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static BYTE CardType = 0U;
static volatile uint8_t sd_diag_stage = SD_DIAG_STAGE_IDLE;
static volatile uint8_t sd_diag_last_cmd = 0U;
static volatile uint8_t sd_diag_last_r1 = 0xFFU;
static volatile uint8_t sd_diag_last_token = 0xFFU;

/* USER CODE BEGIN PRIVATE_FUNCTION_PROTO */
static uint8_t sd_spi_txrx(uint8_t tx);
static void sd_spi_rx(uint8_t *buff, UINT len);
static void sd_spi_tx(const uint8_t *buff, UINT len);
static int sd_spi_set_prescaler(uint32_t prescaler);
static void sd_select(void);
static void sd_deselect(void);
static int sd_wait_ready(uint32_t timeout_ms);
static int sd_select_wait_ready(void);
static int rcvr_datablock(BYTE *buff, UINT btr);
static int xmit_datablock(const BYTE *buff, BYTE token);
static BYTE send_cmd(BYTE cmd, DWORD arg);
static int sd_read_single_block(DWORD sector, BYTE *buff);
/* USER CODE END PRIVATE_FUNCTION_PROTO */

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

uint8_t USER_SD_GetDiagStage(void)
{
  return sd_diag_stage;
}

uint8_t USER_SD_GetDiagLastCmd(void)
{
  return sd_diag_last_cmd;
}

uint8_t USER_SD_GetDiagLastR1(void)
{
  return sd_diag_last_r1;
}

uint8_t USER_SD_GetDiagLastToken(void)
{
  return sd_diag_last_token;
}

uint8_t USER_SD_GetCardType(void)
{
  return CardType;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
      sd_diag_stage = SD_DIAG_STAGE_INIT_START;
      sd_diag_last_cmd = 0U;
      sd_diag_last_r1 = 0xFFU;
      sd_diag_last_token = 0xFFU;

    BYTE command;
    BYTE ocr[4];
    BYTE card_type = 0U;
    uint32_t start_tick;

    if (pdrv != 0U)
    {
      return STA_NOINIT;
    }

    /* SD cards require a slow SPI clock during idle/identification. */
    if (!sd_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256))
    {
      Stat = STA_NOINIT;
      return Stat;
    }

    Stat = STA_NOINIT;

    /* Ensure at least 74 clocks with CS high before first command. */
    sd_deselect();
    for (uint8_t i = 0U; i < 10U; i++)
    {
      (void) sd_spi_txrx(0xFFU);
    }

    if (send_cmd(CMD0, 0U) == 1U)
    {
      /* SD v2 path: check operating voltage range. */
      if (send_cmd(CMD8, 0x1AAU) == 1U)
      {
        ocr[0] = sd_spi_txrx(0xFFU);
        ocr[1] = sd_spi_txrx(0xFFU);
        ocr[2] = sd_spi_txrx(0xFFU);
        ocr[3] = sd_spi_txrx(0xFFU);

        if ((ocr[2] == 0x01U) && (ocr[3] == 0xAAU))
        {
          start_tick = HAL_GetTick();
          while ((HAL_GetTick() - start_tick) < SD_INIT_TIMEOUT_MS)
          {
            if (send_cmd(ACMD41, 1UL << 30) == 0U)
            {
              break;
            }
          }

          if ((send_cmd(CMD58, 0U) == 0U) && ((HAL_GetTick() - start_tick) < SD_INIT_TIMEOUT_MS))
          {
            ocr[0] = sd_spi_txrx(0xFFU);
            ocr[1] = sd_spi_txrx(0xFFU);
            ocr[2] = sd_spi_txrx(0xFFU);
            ocr[3] = sd_spi_txrx(0xFFU);
            card_type = (ocr[0] & 0x40U) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
          }
          else if ((HAL_GetTick() - start_tick) >= SD_INIT_TIMEOUT_MS)
          {
            sd_diag_stage = SD_DIAG_STAGE_INIT_ACMD41_TIMEOUT;
          }
          else
          {
            sd_diag_stage = SD_DIAG_STAGE_INIT_CMD58_FAIL;
          }
        }
        else
        {
          sd_diag_stage = SD_DIAG_STAGE_INIT_CMD8_FAIL;
        }
      }
      else
      {
        /* SD v1 or MMC path. */
        if (send_cmd(ACMD41, 0U) <= 1U)
        {
          card_type = CT_SD1;
          command = ACMD41;
        }
        else
        {
          card_type = CT_MMC;
          command = CMD1;
        }

        start_tick = HAL_GetTick();
        while ((HAL_GetTick() - start_tick) < SD_INIT_TIMEOUT_MS)
        {
          if (send_cmd(command, 0U) == 0U)
          {
            break;
          }
        }

        if ((HAL_GetTick() - start_tick) >= SD_INIT_TIMEOUT_MS)
        {
          sd_diag_stage = SD_DIAG_STAGE_INIT_ACMD41_TIMEOUT;
          card_type = 0U;
        }
        else if (send_cmd(CMD16, 512U) != 0U)
        {
          sd_diag_stage = SD_DIAG_STAGE_INIT_CMD16_FAIL;
          card_type = 0U;
        }
      }
    }
    else
    {
      sd_diag_stage = SD_DIAG_STAGE_INIT_CMD0_FAIL;
    }

    CardType = card_type;
    sd_deselect();

    if (card_type != 0U)
    {
      Stat &= (DSTATUS) ~STA_NOINIT;
      /* Keep runtime clock conservative until wiring is proven stable. */
      (void) sd_spi_set_prescaler(SPI_BAUDRATEPRESCALER_64);
      sd_diag_stage = SD_DIAG_STAGE_INIT_OK;
    }
    else
    {
      Stat = STA_NOINIT;
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    if (pdrv != 0U)
    {
      return STA_NOINIT;
    }

    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    DWORD read_addr = sector;

    if ((pdrv != 0U) || (count == 0U))
    {
      return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
      return RES_NOTRDY;
    }

    if ((CardType & CT_BLOCK) == 0U)
    {
      read_addr = sector * 512U;
    }

    if (count == 1U)
    {
      if (sd_read_single_block(read_addr, buff))
      {
        count = 0U;
      }
    }
    else
    {
      if (send_cmd(CMD18, read_addr) == 0U)
      {
        do
        {
          if (!rcvr_datablock(buff, 512U))
          {
            sd_diag_stage = SD_DIAG_STAGE_READ_TOKEN_FAIL;
            break;
          }
          buff += 512U;
        } while (--count > 0U);

        (void) send_cmd(CMD12, 0U);
      }
      else
      {
        sd_diag_stage = SD_DIAG_STAGE_READ_CMD_FAIL;
      }
    }

    sd_deselect();
    return (count == 0U) ? RES_OK : RES_ERROR;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
    if ((pdrv != 0U) || (count == 0U))
    {
      return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
      return RES_NOTRDY;
    }

    if (Stat & STA_PROTECT)
    {
      return RES_WRPRT;
    }

    if ((CardType & CT_BLOCK) == 0U)
    {
      sector *= 512U;
    }

    if (count == 1U)
    {
      if (send_cmd(CMD24, sector) == 0U)
      {
        if (xmit_datablock(buff, 0xFEU))
        {
          count = 0U;
        }
        else
        {
          sd_diag_stage = SD_DIAG_STAGE_WRITE_TOKEN_FAIL;
        }
      }
      else
      {
        sd_diag_stage = SD_DIAG_STAGE_WRITE_CMD_FAIL;
      }
    }
    else
    {
      if (CardType & (CT_SD1 | CT_SD2))
      {
        (void) send_cmd(ACMD23, count);
      }

      if (send_cmd(CMD25, sector) == 0U)
      {
        do
        {
          if (!xmit_datablock(buff, 0xFCU))
          {
            sd_diag_stage = SD_DIAG_STAGE_WRITE_TOKEN_FAIL;
            break;
          }
          buff += 512U;
        } while (--count > 0U);

        if (!xmit_datablock(0U, 0xFDU))
        {
          sd_diag_stage = SD_DIAG_STAGE_WRITE_TOKEN_FAIL;
          count = 1U;
        }
      }
      else
      {
        sd_diag_stage = SD_DIAG_STAGE_WRITE_CMD_FAIL;
      }
    }

    sd_deselect();
    return (count == 0U) ? RES_OK : RES_ERROR;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    BYTE n;
    BYTE csd[16];
    WORD csize;

    if (pdrv != 0U)
    {
      return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
      return RES_NOTRDY;
    }

    switch (cmd)
    {
      case CTRL_SYNC:
        if (sd_select_wait_ready())
        {
          sd_deselect();
          res = RES_OK;
        }
        break;

      case GET_SECTOR_COUNT:
        if ((send_cmd(CMD9, 0U) == 0U) && rcvr_datablock(csd, 16U))
        {
          if ((csd[0] >> 6) == 1U)
          {
            csize = (WORD) csd[9] + ((WORD) csd[8] << 8) + 1U;
            *(DWORD *) buff = (DWORD) csize << 10;
          }
          else
          {
            n = (BYTE) ((csd[5] & 15U) + ((csd[10] & 128U) >> 7) + ((csd[9] & 3U) << 1) + 2U);
            csize = (WORD) ((csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3U) << 10) + 1U);
            *(DWORD *) buff = (DWORD) csize << (n - 9U);
          }
          res = RES_OK;
        }
        break;

      case GET_SECTOR_SIZE:
        *(WORD *) buff = 512U;
        res = RES_OK;
        break;

      case GET_BLOCK_SIZE:
        if (CardType & CT_SD2)
        {
          *(DWORD *) buff = 128U;
          res = RES_OK;
        }
        else if ((send_cmd(CMD9, 0U) == 0U) && rcvr_datablock(csd, 16U))
        {
          if (CardType & CT_SD1)
          {
            *(DWORD *) buff = (DWORD) ((((csd[10] & 63U) << 1) + ((csd[11] & 128U) >> 7) + 1U)
                                 << ((csd[13] >> 6) - 1U));
          }
          else
          {
            *(DWORD *) buff = (DWORD) (((csd[10] & 124U) >> 2) + 1U)
                               * (DWORD) ((((csd[11] & 3U) << 3) | ((csd[11] & 224U) >> 5)) + 1U);
          }
          res = RES_OK;
        }
        break;

      case MMC_GET_TYPE:
        *(BYTE *) buff = CardType;
        res = RES_OK;
        break;

      case MMC_GET_CSD:
        if ((send_cmd(CMD9, 0U) == 0U) && rcvr_datablock((BYTE *) buff, 16U))
        {
          res = RES_OK;
        }
        break;

      case MMC_GET_CID:
        if ((send_cmd(CMD10, 0U) == 0U) && rcvr_datablock((BYTE *) buff, 16U))
        {
          res = RES_OK;
        }
        break;

      case MMC_GET_OCR:
        if (send_cmd(CMD58, 0U) == 0U)
        {
          for (n = 0U; n < 4U; n++)
          {
            ((BYTE *) buff)[n] = sd_spi_txrx(0xFFU);
          }
          res = RES_OK;
        }
        break;

      case MMC_GET_SDSTAT:
        if ((send_cmd(ACMD13, 0U) == 0U) && rcvr_datablock((BYTE *) buff, 64U))
        {
          res = RES_OK;
        }
        break;

      default:
        sd_diag_stage = SD_DIAG_STAGE_IOCTL_FAIL;
        res = RES_PARERR;
        break;
    }

    sd_deselect();
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS */
static uint8_t sd_spi_txrx(uint8_t tx)
{
  uint8_t rx = 0xFFU;

  if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U, SD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    return 0xFFU;
  }

  return rx;
}

static void sd_spi_rx(uint8_t *buff, UINT len)
{
  while (len > 0U)
  {
    *buff = sd_spi_txrx(0xFFU);
    buff++;
    len--;
  }
}

static void sd_spi_tx(const uint8_t *buff, UINT len)
{
  while (len > 0U)
  {
    (void) sd_spi_txrx(*buff);
    buff++;
    len--;
  }
}

static int sd_spi_set_prescaler(uint32_t prescaler)
{
  if (hspi2.Init.BaudRatePrescaler == prescaler)
  {
    return 1;
  }

  hspi2.Init.BaudRatePrescaler = prescaler;

  if (HAL_SPI_DeInit(&hspi2) != HAL_OK)
  {
    return 0;
  }

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    return 0;
  }

  return 1;
}

static void sd_select(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void sd_deselect(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  (void) sd_spi_txrx(0xFFU);
}

static int sd_wait_ready(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  do
  {
    if (sd_spi_txrx(0xFFU) == 0xFFU)
    {
      return 1;
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return 0;
}

static int sd_select_wait_ready(void)
{
  sd_select();
  (void) sd_spi_txrx(0xFFU);
  if (sd_wait_ready(SD_READY_TIMEOUT_MS))
  {
    return 1;
  }

  sd_deselect();
  return 0;
}

static int rcvr_datablock(BYTE *buff, UINT btr)
{
  BYTE token;
  uint32_t start_tick;

  start_tick = HAL_GetTick();
  do
  {
    token = sd_spi_txrx(0xFFU);
  } while ((token == 0xFFU) && ((HAL_GetTick() - start_tick) < SD_READY_TIMEOUT_MS));

  if (token != 0xFEU)
  {
    sd_diag_last_token = token;
    return 0;
  }

  sd_spi_rx(buff, btr);
  (void) sd_spi_txrx(0xFFU);
  (void) sd_spi_txrx(0xFFU);

  return 1;
}

static int xmit_datablock(const BYTE *buff, BYTE token)
{
  BYTE response;

  if (!sd_wait_ready(SD_READY_TIMEOUT_MS))
  {
    return 0;
  }

  (void) sd_spi_txrx(token);

  if (token == 0xFDU)
  {
    return 1;
  }

  sd_spi_tx(buff, 512U);
  (void) sd_spi_txrx(0xFFU);
  (void) sd_spi_txrx(0xFFU);
  response = sd_spi_txrx(0xFFU);
  sd_diag_last_token = response;

  return ((response & 0x1FU) == 0x05U) ? 1 : 0;
}

static BYTE send_cmd(BYTE cmd, DWORD arg)
{
  BYTE n;
  BYTE res;
  BYTE crc;

  if (cmd & 0x80U)
  {
    cmd &= 0x7FU;
    res = send_cmd(CMD55, 0U);
    if (res > 1U)
    {
      return res;
    }
  }

  sd_diag_last_cmd = cmd;

  sd_deselect();
  if (!sd_select_wait_ready())
  {
    return 0xFFU;
  }

  crc = 0x01U;
  if (cmd == CMD0)
  {
    crc = 0x95U;
  }
  if (cmd == CMD8)
  {
    crc = 0x87U;
  }

  (void) sd_spi_txrx((uint8_t) (0x40U | cmd));
  (void) sd_spi_txrx((uint8_t) (arg >> 24));
  (void) sd_spi_txrx((uint8_t) (arg >> 16));
  (void) sd_spi_txrx((uint8_t) (arg >> 8));
  (void) sd_spi_txrx((uint8_t) arg);
  (void) sd_spi_txrx(crc);

  if (cmd == CMD12)
  {
    (void) sd_spi_txrx(0xFFU);
  }

  n = 10U;
  do
  {
    res = sd_spi_txrx(0xFFU);
  } while ((res & 0x80U) && (--n > 0U));

  sd_diag_last_r1 = res;

  return res;
}

static int sd_read_single_block(DWORD read_addr, BYTE *buff)
{
  BYTE r1;
  uint8_t attempt;

  for (attempt = 0U; attempt < SD_READ_RETRY_COUNT; attempt++)
  {
    r1 = send_cmd(CMD17, read_addr);
    if (r1 == 0U)
    {
      if (rcvr_datablock(buff, 512U))
      {
        return 1;
      }

      sd_diag_stage = SD_DIAG_STAGE_READ_TOKEN_FAIL;
      continue;
    }

    sd_diag_stage = SD_DIAG_STAGE_READ_CMD_FAIL;

    /* Some cards report PARAMETER_ERROR if block/byte mode was misdetected.
     * Retry once with alternate addressing and latch the working mode.
     */
    if ((r1 & 0x10U) != 0U)
    {
      DWORD alt_addr;

      if ((CardType & CT_BLOCK) != 0U)
      {
        alt_addr = read_addr * 512U;
      }
      else
      {
        alt_addr = read_addr / 512U;
      }

      r1 = send_cmd(CMD17, alt_addr);
      if ((r1 == 0U) && rcvr_datablock(buff, 512U))
      {
        if ((CardType & CT_BLOCK) != 0U)
        {
          CardType &= (BYTE) ~CT_BLOCK;
        }
        else
        {
          CardType |= CT_BLOCK;
        }

        return 1;
      }
    }
  }

  return 0;
}
/* USER CODE END PRIVATE_FUNCTIONS */

