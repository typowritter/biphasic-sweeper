/**
  ******************************************************************************
  * @file    sdram.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-05-23
  * @brief   driver for onboard 2x32M SDRAM (W9825G6KH)
  ******************************************************************************
  */

#include "sdram.h"

static __IO uint32_t *sdram_base_addr = (uint32_t *)SDRAM_BANK_ADDR;

void sdram_init()
{
  FMC_SDRAM_CommandTypeDef Command = {0};

  /* enable clock to SDRAM */
  Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget = FMC_COMMAND_TARGET_BANK;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  if (HAL_SDRAM_SendCommand(&sdram_dev, &Command, 0xFFFF) != HAL_OK)
  {
    Error_Handler();
  }

  /* at least 100us */
  HAL_Delay(1);

  /* precharge */
  Command.CommandMode = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget = FMC_COMMAND_TARGET_BANK;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  if (HAL_SDRAM_SendCommand(&sdram_dev, &Command, 0xFFFF) != HAL_OK)
  {
    Error_Handler();
  }

  /* auto-refresh */
  Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget = FMC_COMMAND_TARGET_BANK;
  Command.AutoRefreshNumber = 8;
  Command.ModeRegisterDefinition = 0;

  if (HAL_SDRAM_SendCommand(&sdram_dev, &Command, 0xFFFF) != HAL_OK)
  {
    Error_Handler();
  }

  /* sdram register */
  uint32_t tmpr = (uint32_t)
           SDRAM_MODEREG_BURST_LENGTH_1
         | SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL
         | SDRAM_MODEREG_CAS_LATENCY_3
         | SDRAM_MODEREG_OPERATING_MODE_STANDARD
         | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget = FMC_COMMAND_TARGET_BANK;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = tmpr;

  if (HAL_SDRAM_SendCommand(&sdram_dev, &Command, 0xFFFF) != HAL_OK)
  {
    Error_Handler();
  }

  /* refresh counter */
  /* refresh command interval = 64ms/8192 rows = 7.8125us */
  /* COUNT = (7.8125us x fSDCLK) - 20 */
  /* where fSDCLK = 120MHz */
  HAL_SDRAM_ProgramRefreshRate(&sdram_dev, 918);
}

/**
 * write words to sdram
 *
 * @param p_buffer  -- pointer to the data buffer
 * @param w_addr    -- targeted write address in sdram
 * @param w_size    -- size of the data, in words
 */
void sdram_write(uint32_t* p_buffer, uint32_t w_addr, uint32_t w_size)
{
  HAL_SDRAM_WriteProtection_Disable(&sdram_dev);
  while (HAL_SDRAM_GetState(&sdram_dev) != HAL_SDRAM_STATE_READY);

  for (uint32_t offset = w_addr; w_size != 0; w_size--)
  {
    sdram_base_addr[offset] = *p_buffer++;
    offset += 4;
  }
}

/**
 * read words from sdram to buffer
 *
 * @param p_buffer  -- pointer to the store buffer
 * @param r_addr    -- targeted read address in sdram
 * @param r_size    -- size of the data, in words
 */
void sdram_read(uint32_t* p_buffer, uint32_t r_addr, uint32_t r_size)
{
  while (HAL_SDRAM_GetState(&sdram_dev) != HAL_SDRAM_STATE_READY);

  for (uint32_t offset = r_addr; r_size != 0; r_size--)
  {
    *p_buffer++ = sdram_base_addr[offset];
    offset += 4;
  }
}

#ifdef DEBUG
#include "tty.h"
#include "utils.h"
#define sdram_msg(...) DEBUG_GEN(tty_print, "SDRAM: ", __VA_ARGS__)

void sdram_test()
{
  uint32_t testdata[16] = {
    0x12abe678, 0x87cfe321, 0x18ea2749, 0x7abd9274,
    0x34242356, 0x69034543, 0x54367688, 0x32568996,
    0xa8896ce3, 0x23794898, 0x219082a9, 0xff993231,
    0x8989abce, 0x29039027, 0x1987acdf, 0x345080ac,
  };

  uint32_t read_buffer[16] = {0};

  sdram_write(&testdata[0], 0, 16);
  sdram_read(&read_buffer[0], 0, 16);

  int err = 0;
  for (int i = 0; i < 16; i++)
  {
    if (testdata[i] != read_buffer[i])
    {
      sdram_msg("mismatch on [%02d] = 0x%x, expecting 0x%x\r\n",
                i, read_buffer[i], testdata[i]);
      err++;
    }
  }

  sdram_msg("errors = %d\r\n", err);
}

#else
void sdram_test()
{
}

#endif
