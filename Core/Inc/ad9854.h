/**
  ******************************************************************************
  * @file    ad9854.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-04
  * @brief   driver for ad9854
  ******************************************************************************
  * @attention
  *
  * We use a custom board where the S/P SELECT pin is tied high
  * so the serial mode is not implemented
  * Also not the reg-read function.
  *
  ******************************************************************************
  */

#ifndef __AD9854_H
#define __AD9854_H

#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-const-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

#include "main.h"
#include "spi.h"
#include "gpio_wrapper.h"
#include "utils.h"

/* BEGIN project specific setups
 * keep those synchronized with global settings */
#define USE_PARALLEL       0
#define ad9854_dev         hspi3
#define ad9854_spi_timeout 100000
#define ad9854_sysclk      30000000

#if USE_PARALLEL
  DEF_GPIO(ad9854_pin_wr,     AD9854_WR_GPIO_Port,    AD9854_WR_Pin);
  // DEF_GPIO(ad9854_pin_rd,     AD9854_RD_GPIO_Port,    AD9854_RD_Pin);
  DEF_GPIO(ad9854_pin_rst,    AD9854_RST_GPIO_Port,   AD9854_RST_Pin);
  DEF_GPIO(ad9854_pin_osk,    AD9854_OSK_GPIO_Port,   AD9854_OSK_Pin);
  DEF_GPIO(ad9854_pin_fsk,    AD9854_FSK_GPIO_Port,   AD9854_FSK_Pin);
  DEF_GPIO(ad9854_pin_udclk,  AD9854_UDCL_GPIO_Port,  AD9854_UDCL_Pin);
  DEF_GPIO_GROUP(ad9854_par_addr, GPIOJ, 6, 2);
  DEF_GPIO_GROUP(ad9854_par_data, GPIOJ, 8, 8);
#else
  DEF_GPIO(ad9854_pin_cs,     AD9854_CS_GPIO_Port,    AD9854_CS_Pin);
  DEF_GPIO(ad9854_pin_iorst,  AD9854_IORST_GPIO_Port, AD9854_IORST_Pin);
  DEF_GPIO(ad9854_pin_rst,    AD9854_RST_GPIO_Port,   AD9854_RST_Pin);
  DEF_GPIO(ad9854_pin_osk,    AD9854_OSK_GPIO_Port,   AD9854_OSK_Pin);
  DEF_GPIO(ad9854_pin_fsk,    AD9854_FSK_GPIO_Port,   AD9854_FSK_Pin);
  DEF_GPIO(ad9854_pin_udclk,  AD9854_UDCL_GPIO_Port,  AD9854_UDCL_Pin);
#endif
#undef DEF_GPIO
#undef DEF_GPIO_GROUP

/* END project specific setups */


typedef struct
{
  /* the physical serial address in the chip */
  const uint8_t s_addr;
  /* the physical parallel address in the chip */
  const uint8_t p_addr;
  /* the current value stored locally for transmission */
  uint64_t value;
  /* number of bytes of the register */
  const uint8_t size;
} ad9854_register;

typedef struct
{
  ad9854_register* reg;
  uint8_t bits;
  uint8_t offset;
} ad9854_register_bit;

typedef struct
{
  ad9854_register par1;           // Phase Adjust Register 1
  ad9854_register par2;           // Phase Adjust Register 1
  ad9854_register ftw1;           // Frequency Tuning Word 1
  ad9854_register ftw2;           // Frequency Tuning Word 2
  ad9854_register dfw;            // Delta frequency word
  ad9854_register update_clk;     // Update clock
  ad9854_register ramp_rate_clk;  // Ramp rate clock
  ad9854_register cr;             // Control Register
  ad9854_register osk_i_mult;     // Output shaped keying I multiplier
  ad9854_register osk_q_mult;     // Output shaped keying Q multiplier
  ad9854_register osk_ramp_rate;  // Output shaped keying ramp rate
  ad9854_register qdac;           // QDAC
} ad9854_registers;


/* global struct with shadow registers of the real values which are
 * currently in the ADC */
extern ad9854_registers ad9854_regs;

#define DEF_REG_BIT(_name, _reg, _bits, _offset)                                \
  static const ad9854_register_bit ad9854_##_name = { .reg = &ad9854_regs._reg, \
                                                      .bits = _bits,            \
                                                      .offset = _offset }

DEF_REG_BIT(comp_pwd,      cr, 1, 28);
DEF_REG_BIT(qdac_pwd,      cr, 1, 26);
DEF_REG_BIT(dac_pwd,       cr, 1, 25);
DEF_REG_BIT(dig_pwd,       cr, 1, 24);
DEF_REG_BIT(pll_range,     cr, 1, 22);
DEF_REG_BIT(pll_bypass,    cr, 1, 21);
DEF_REG_BIT(pll_mult,      cr, 5, 16);
DEF_REG_BIT(clr_acc1,      cr, 1, 15);
DEF_REG_BIT(clr_acc2,      cr, 1, 14);
DEF_REG_BIT(triangle,      cr, 1, 13);
DEF_REG_BIT(src_qdac,      cr, 1, 12);
DEF_REG_BIT(mode,          cr, 3, 9);
DEF_REG_BIT(updclk,        cr, 1, 8);
DEF_REG_BIT(invsinc_byp,   cr, 1, 6);
DEF_REG_BIT(osk_en,        cr, 1, 5);
DEF_REG_BIT(osk_int,       cr, 1, 4);
DEF_REG_BIT(lsb_first,     cr, 1, 1);
DEF_REG_BIT(sdo_cr,        cr, 1, 0);

#undef DEF_REG_BIT

typedef enum {
  ad9854_mode_single    = 0x00,
  ad9854_mode_fsk       = 0x01,
  ad9854_mode_ramp_fsk  = 0x02,
  ad9854_mode_chirp     = 0x03,
  ad9854_mode_bpsk      = 0x04,
} ad9854_mode_t;

typedef enum {
  ad9854_updclk_external = 0,
  ad9854_updclk_internal = 1,
} ad9854_updclk_t;

static INLINE void      ad9854_select();
static INLINE void      ad9854_unselect();
static INLINE void      ad9854_set_bits(ad9854_register_bit field, uint8_t value);
static INLINE uint8_t   ad9854_get_bits(ad9854_register_bit field);
static INLINE void      ad9854_update_reg(ad9854_register* reg);
static INLINE void      ad9854_update_bits(ad9854_register_bit field);

void ad9854_init();
void ad9854_reset();
void freq_convert(uint64_t freq);
void amp_convert(uint16_t amp);

/* parallel and serial interface */
#if USE_PARALLEL
uint8_t ad9854_read_byte(uint8_t addr);
uint64_t ad9854_read_parallel(ad9854_register* reg);
void ad9854_write_byte(uint8_t addr, uint8_t data);
void ad9854_write_parallel(ad9854_register* reg, uint64_t value);
#else
uint64_t ad9854_read_serial(ad9854_register* reg);
void ad9854_write_serial(ad9854_register* reg, uint64_t value);
#endif

/** implementation starts here */
static INLINE void
ad9854_select()
{
#if USE_PARALLEL
  undefined();
#else
  gpio_set_low(ad9854_pin_cs);
#endif
}

static INLINE void
ad9854_unselect()
{
#if USE_PARALLEL
  undefined();
#else
  gpio_set_high(ad9854_pin_cs);
#endif
}

static INLINE void
ad9854_set_bits(ad9854_register_bit field, uint8_t value)
{
  /* convert the numbers of bits in a mask with matching length */
  const uint32_t mask = ((uint32_t)1 << field.bits) - 1;
  /* clear affected bits */
  field.reg->value &= ~(mask << field.offset);
  /* set affected bits */
  field.reg->value |= ((value & mask) << field.offset);
}

static INLINE uint8_t
ad9854_get_bits(ad9854_register_bit field)
{
  /* convert the numbers of bits in a mask with matching length */
  const uint8_t mask = ((uint8_t)1 << field.bits) - 1;
  return (field.reg->value >> field.offset) & mask;
}

static INLINE void
ad9854_update_reg(ad9854_register* reg)
{
#if USE_PARALLEL
  ad9854_write_parallel(reg, reg->value);
#else
  ad9854_write_serial(reg, reg->value);
#endif
}

static INLINE void
ad9854_update_bits(ad9854_register_bit field)
{
  ad9854_update_reg(field.reg);
}

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __AD9854_H */
