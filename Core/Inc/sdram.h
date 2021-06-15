/**
  ******************************************************************************
  * @file    sdram.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-05-23
  * @brief   driver for onboard 2x32M SDRAM (W9825G6KH)
  ******************************************************************************
  */

#ifndef __SDRAM_H
#define __SDRAM_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "fmc.h"

/* 64MB@32bit = 2 x 32MB@16bit combined */
#define sdram_size                0x4000000
#define sdram_dev                 hsdram2

/* FMC configuration specific */
#define FMC_BANK_SDRAM            FMC_Bank2_SDRAM
#define FMC_COMMAND_TARGET_BANK   FMC_SDRAM_CMD_TARGET_BANK2
#define SDRAM_BANK_ADDR           ((uint32_t)0xD0000000)

/* SDRAM register configuration */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)


/* this should be called after FMC is initialized */
void sdram_init();

/**
 * write words to sdram
 *
 * @param p_buffer  -- pointer to the data buffer
 * @param w_addr    -- targeted write address in sdram
 * @param w_size    -- size of the data, in words
 */
void sdram_write(uint32_t* p_buffer, uint32_t w_addr, uint32_t w_size);

/**
 * read words from sdram to buffer
 *
 * @param p_buffer  -- pointer to the store buffer
 * @param r_addr    -- targeted read address in sdram
 * @param r_size    -- size of the data, in words
 */
void sdram_read(uint32_t* p_buffer, uint32_t r_addr, uint32_t r_size);

/* sdram read and write test */
void sdram_test();

#ifdef  __cplusplus
}
#endif
#endif /* __SDRAM_H */
