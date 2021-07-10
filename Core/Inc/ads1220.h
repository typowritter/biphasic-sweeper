/**
  ******************************************************************************
  * @file    ads1220.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-03
  * @brief   driver for ads1220
  ******************************************************************************
  */

#ifndef __ADS1220_H
#define __ADS1220_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "gpio_wrapper.h"
#include "spi.h"
#include "utils.h"

/* BEGIN project specific setups
 * keep those synchronized with global settings */
#define ads1220_dev         hspi1
#define ads1220_spi_timeout 100000
DEF_GPIO(ads1220_pin_cs,   ADS1220_CS_GPIO_Port,   ADS1220_CS_Pin);
DEF_GPIO(ads1220_pin_drdy, ADS1220_DRDY_GPIO_Port, ADS1220_DRDY_Pin);

/* END project specific setups */


typedef struct
{
  /* the physical address in the chip */
  const uint8_t addr;
  /* the current value stored locally for transmission */
  uint8_t value;
} ads1220_register;

typedef struct
{
  ads1220_register* reg;
  uint8_t bits;
  uint8_t offset;
} ads1220_register_bit;


/* global struct with shadow registers of the real values which are
 * currently in the ADC */
extern ads1220_register ads1220_regs[];

#define DEF_REG_BIT(_name, _reg, _bits, _offset)                                    \
  static const ads1220_register_bit ads1220_##_name = { .reg = &ads1220_regs[_reg], \
                                                        .bits = _bits,              \
                                                        .offset = _offset }

/* Register 0 */
DEF_REG_BIT(mux,        0, 4, 4);    // Input multiplexer configuration
DEF_REG_BIT(gain,       0, 3, 1);    // Gain configuration
DEF_REG_BIT(pga_bypass, 0, 1, 0);    // Disables and bypasses the PGA

/* Register 1 */
DEF_REG_BIT(dr,         1, 3, 5);    // Data rate
DEF_REG_BIT(mode,       1, 2, 3);    // Operating mode
DEF_REG_BIT(cm,         1, 1, 2);    // Conversion mode
DEF_REG_BIT(ts,         1, 1, 1);    // Temperature sensor mode
DEF_REG_BIT(bcs,        1, 1, 0);    // Burn-out current sources

/* Register 2 */
DEF_REG_BIT(vref,       2, 2, 6);    // Voltage reference selection
DEF_REG_BIT(rejection,  2, 2, 4);    // FIR filter configuration
DEF_REG_BIT(psw,        2, 1, 3);    // Low-side power switch configuration
DEF_REG_BIT(idac,       2, 3, 0);    // IDAC current setting

/* Register 3 */
DEF_REG_BIT(i1mux,      3, 3, 5);    // IDAC1 routing configuration
DEF_REG_BIT(i2mux,      3, 3, 2);    // IDAC2 routing configuration
DEF_REG_BIT(drdym,      3, 1, 1);    // DRDY mode

#undef DEF_REG_BIT

typedef enum {
  ads1220_cmd_reset   = 0x06,
  ads1220_cmd_start   = 0x08,
  ads1220_cmd_pwrdwn  = 0x02,
  ads1220_cmd_rdata   = 0x10,
  ads1220_cmd_rreg    = 0x20,
  ads1220_cmd_wreg    = 0x40,
} ads1220_cmd_t;

typedef enum {
  ads1220_ain_p0n1   = 0x00,
  ads1220_ain_p0n2   = 0x01,
  ads1220_ain_p0n3   = 0x02,
  ads1220_ain_p1n2   = 0x03,
  ads1220_ain_p1n3   = 0x04,
  ads1220_ain_p2n3   = 0x05,
  ads1220_ain_p1n0   = 0x06,
  ads1220_ain_p3n2   = 0x07,
  ads1220_ain_p0ns   = 0x08,
  ads1220_ain_p1ns   = 0x09,
  ads1220_ain_p2ns   = 0x0a,
  ads1220_ain_p3ns   = 0x0b,
  ads1220_ain_ref_4  = 0x0c,  // (VREFPx – VREFNx) / 4 monitor (PGA bypassed)
  ads1220_ain_fs_4   = 0x0d,  // (AVDD – AVSS) / 4 monitor (PGA bypassed)
  ads1220_ain_short  = 0x0e,  // AINP and AINN shorted to (AVDD + AVSS) / 2
} ads1220_ain_t;

typedef enum {
  ads1220_gain_x1    = 0x00,
  ads1220_gain_x2    = 0x01,
  ads1220_gain_x4    = 0x02,
  /* Gains 1, 2, and 4 can be used without the PGA */
  ads1220_gain_x8    = 0x03,
  ads1220_gain_x16   = 0x04,
  ads1220_gain_x32   = 0x05,
  ads1220_gain_x64   = 0x06,
  ads1220_gain_x128  = 0x07,
} ads1220_gain_t;

typedef enum {
  ads1220_dr_normal_x20     = 0x00,
  ads1220_dr_normal_x45     = 0x01,
  ads1220_dr_normal_x90     = 0x02,
  ads1220_dr_normal_x175    = 0x03,
  ads1220_dr_normal_x330    = 0x04,
  ads1220_dr_normal_x600    = 0x05,
  ads1220_dr_normal_x1000   = 0x06,
} ads1220_dr_normal_t;

typedef enum {
  ads1220_dr_dutycycle_x5     = 0x00,
  ads1220_dr_dutycycle_x11_25 = 0x01,
  ads1220_dr_dutycycle_x22_5  = 0x02,
  ads1220_dr_dutycycle_x44    = 0x03,
  ads1220_dr_dutycycle_x82_5  = 0x04,
  ads1220_dr_dutycycle_x150   = 0x05,
  ads1220_dr_dutycycle_x250   = 0x06,
} ads1220_dr_dutycycle_t;

typedef enum {
  ads1220_dr_turbo_x40    = 0x00,
  ads1220_dr_turbo_x90    = 0x01,
  ads1220_dr_turbo_x180   = 0x02,
  ads1220_dr_turbo_x350   = 0x03,
  ads1220_dr_turbo_x660   = 0x04,
  ads1220_dr_turbo_x1200  = 0x05,
  ads1220_dr_turbo_x2000  = 0x06,
} ads1220_dr_turbo_t;

typedef enum {
  ads1220_mode_normal     = 0x00,
  ads1220_mode_dutycycle  = 0x01,
  ads1220_mode_turbo      = 0x02,
} ads1220_mode_t;

typedef enum {
  ads1220_conv_single     = 0,
  ads1220_conv_continuous = 1,
} ads1220_conv_t;

typedef enum {
  ads1220_vref_internal   = 0x00,
  ads1220_vref_p0n0       = 0x01,
  ads1220_vref_p1n1       = 0x02,
  ads1220_vref_analog_fs  = 0x03,
} ads1220_vref_t;

typedef enum {
  ads1220_rejection_off   = 0x00,
  ads1220_rejection_50_60 = 0x01,
  ads1220_rejection_50    = 0x02,
  ads1220_rejection_60    = 0x03,
} ads1220_rejection_t;

typedef enum {
  ads1220_psw_open  = 0,
  ads1220_psw_auto  = 1,
} ads1220_psw_t;

typedef enum {
  ads1220_idac_off    = 0x00,
  ads1220_idac_10mu   = 0x01,
  ads1220_idac_50mu   = 0x02,
  ads1220_idac_100mu  = 0x03,
  ads1220_idac_250mu  = 0x04,
  ads1220_idac_500mu  = 0x05,
  ads1220_idac_1000mu = 0x06,
  ads1220_idac_1500mu = 0x07,
} ads1220_idac_t;

typedef enum {
  ads1220_imux_disable  = 0x00,
  ads1220_imux_ain0     = 0x01,
  ads1220_imux_ain1     = 0x02,
  ads1220_imux_ain2     = 0x03,
  ads1220_imux_ain3     = 0x04,
  ads1220_imux_refp0    = 0x05,
  ads1220_imux_refn0    = 0x06,
} ads1220_imux_t;

typedef enum {
  ads1220_drdym_dedicate = 0,
  ads1220_drdym_combined = 1,
} ads1220_drdym_t;


static INLINE void      ads1220_select();
static INLINE void      ads1220_unselect();
static INLINE void      ads1220_set_value(ads1220_register_bit field, uint8_t value);
static INLINE uint8_t   ads1220_get_value(ads1220_register_bit field);
static INLINE void      ads1220_read_reg(ads1220_register* reg);
static INLINE void      ads1220_write_reg(ads1220_register* reg, uint8_t byte);
static INLINE void      ads1220_send_cmd(uint8_t cmd);
static INLINE void      ads1220_update_reg(ads1220_register* reg);
static INLINE void      ads1220_update_matching_reg(ads1220_register_bit field);

void ads1220_init();
void ads1220_test();
void ads1220_reset();
void ads1220_read_conv_data(uint32_t *conv_data);

/**
 * reads multiple registers from device.
 *
 * @param reg   -- the first register to read
 * @param num   -- number of registers to read
 * @param data  -- buffer for received bytes,
                   should at least have a length of num
 *
 * @note: global struct ads1220_regs WILL be updated
 */
void ads1220_read_regs(ads1220_register* reg, uint8_t num);

void ads1220_write_regs(ads1220_register* reg, uint8_t num, uint8_t* data);


/** implementation starts here */
static INLINE void
ads1220_select()
{
  gpio_set_low(ads1220_pin_cs);
}

static INLINE void
ads1220_unselect()
{
  gpio_set_high(ads1220_pin_cs);
}

static INLINE void
ads1220_set_value(ads1220_register_bit field, uint8_t value)
{
  /* convert the numbers of bits in a mask with matching length */
  const uint8_t mask = ((uint8_t)1 << field.bits) - 1;
  /* clear affected bits */
  field.reg->value &= ~(mask << field.offset);
  /* set affected bits */
  field.reg->value |= ((value & mask) << field.offset);
}

static INLINE uint8_t
ads1220_get_value(ads1220_register_bit field)
{
  /* convert the numbers of bits in a mask with matching length */
  const uint8_t mask = ((uint8_t)1 << field.bits) - 1;
  return (field.reg->value >> field.offset) & mask;
}

static INLINE void
ads1220_read_reg(ads1220_register* reg)
{
  ads1220_read_regs(reg, 1);
}

static INLINE void
ads1220_write_reg(ads1220_register* reg, uint8_t byte)
{
  ads1220_write_regs(reg, 1, lit2addr(byte));
}

static INLINE void
ads1220_send_cmd(uint8_t cmd)
{
  HAL_SPI_Transmit(&ads1220_dev, lit2addr(cmd), 1, ads1220_spi_timeout);
}

static INLINE void
ads1220_update_reg(ads1220_register* reg)
{
  ads1220_write_reg(reg, reg->value);
}

static INLINE void
ads1220_update_matching_reg(ads1220_register_bit field)
{
  ads1220_update_reg(field.reg);
}

#ifdef  __cplusplus
}
#endif
#endif /* __ADS1220_H */
