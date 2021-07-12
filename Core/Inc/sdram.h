/**
  ******************************************************************************
  * @file    sdram.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-15
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

/* data length is 32-bit */
typedef uint32_t sdram_data_t;

extern __IO sdram_data_t *sdram_base_addr;

/* this should be called after FMC is initialized */
void sdram_init();

/* sdram read and write test */
void sdram_test();

/**
 * write words to sdram
 *
 * @param p_buffer  -- pointer to the data buffer
 * @param w_addr    -- targeted write address in sdram
 * @param w_size    -- size of the data, in words
 */
void sdram_write(sdram_data_t* p_buffer, uint32_t w_addr, uint32_t w_size);

/**
 * read words from sdram to buffer
 *
 * @param p_buffer  -- pointer to the store buffer
 * @param r_addr    -- targeted read address in sdram
 * @param r_size    -- size of the data, in words
 */
void sdram_read(sdram_data_t* p_buffer, uint32_t r_addr, uint32_t r_size);


#ifdef  __cplusplus
}
#endif
#endif /* __SDRAM_H */
