#ifndef __ADS124S0X_H
#define __ADS124S0X_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "utils.h"

#define ads124s_dev         hspi1
#define ads124s_spi_timeout 100000

enum ads124s_id {
  ADS124S08_ID,
  ADS124S06_ID,
};

typedef struct
{
  /* the physical address in the chip */
  const uint8_t addr;
  /* the current value stored locally for transmission */
  uint8_t value;
  /* may be modified by the device */
  const bool is_volatile;
} ads124s_register;

typedef struct
{
  ads124s_register* reg;
  int bits;
  int offset;
} ads124s_register_bit;

typedef struct
{
  ads124s_register id;
  ads124s_register status;
  ads124s_register inpmux;
  ads124s_register pga;
  ads124s_register datarate;
  ads124s_register ref;
  ads124s_register idacmag;
  ads124s_register idacmux;
  ads124s_register vbias;
  ads124s_register sys;
  ads124s_register ofcal0;
  ads124s_register ofcal1;
  ads124s_register ofcal2;
  ads124s_register fscal0;
  ads124s_register fscal1;
  ads124s_register fscal2;
  ads124s_register gpiodat;
  ads124s_register gpiocon;
} ads124s_registers;


/* global struct with shadow registers of the real values which are
 * currently in the ADC */
extern ads124s_registers ads124s_regs;

#define DEF_REG_BIT(_name, _reg, _bits, _offset)                                    \
  static const ads124s_register_bit ads124s_##_name = { .reg = &ads124s_regs._reg,  \
                                                        .bits = _bits,              \
                                                        .offset = _offset }


DEF_REG_BIT(dev_id,           id,       3, 0);  // Device identifier

DEF_REG_BIT(fl_pwr_on_reset,  status,   1, 7);  // POR flag
DEF_REG_BIT(nrdy,             status,   1, 6);  // Device ready flag
DEF_REG_BIT(fl_p_rail_p,      status,   1, 5);  // Positive PGA output at positive rail flag
DEF_REG_BIT(fl_p_rail_n,      status,   1, 4);  // Positive PGA output at negative rail flag
DEF_REG_BIT(fl_n_rail_p,      status,   1, 3);  // Negative PGA output at positive rail flag
DEF_REG_BIT(fl_n_rail_n,      status,   1, 2);  // Negative PGA output at negative rail flag
DEF_REG_BIT(fl_ref_l1,        status,   1, 1);  // Reference voltage monitor flag, level 1
DEF_REG_BIT(fl_ref_l0,        status,   1, 0);  // Reference voltage monitor flag, level 0

DEF_REG_BIT(muxp,             inpmux,   4, 4);  // Positive ADC input selection
DEF_REG_BIT(muxn,             inpmux,   4, 0);  // Negative ADC input selection

DEF_REG_BIT(pga_conv_delay,   pga,      3, 5);  // Programmable conversion delay selection
DEF_REG_BIT(pga_en,           pga,      2, 3);  // PGA enable
DEF_REG_BIT(pga_gain,         pga,      3, 0);  // PGA gain selection

DEF_REG_BIT(g_chop,           datarate, 1, 7);  // Global chop enable
DEF_REG_BIT(clk_src,          datarate, 1, 6);  // Clock source selection
DEF_REG_BIT(conv_mode,        datarate, 1, 5);  // Conversion mode selection
DEF_REG_BIT(filter_sel,       datarate, 1, 4);  // Digital filter selection
DEF_REG_BIT(datarate_sel,     datarate, 4, 0);  // Data rate selection

DEF_REG_BIT(ref_mon_conf,     ref,      2, 6);  // Reference monitor configuration
DEF_REG_BIT(ref_p_buf,        ref,      1, 5);  // Positive reference buffer bypass
DEF_REG_BIT(ref_n_buf,        ref,      1, 4);  // Negative reference buffer bypass
DEF_REG_BIT(ref_sel,          ref,      2, 2);  // Reference input selection
DEF_REG_BIT(ref_conf,         ref,      2, 0);  // Internal voltage reference configuration

DEF_REG_BIT(fl_rail_en,       idacmag,  1, 7);  // PGA output rail flag enable
DEF_REG_BIT(lowside_psw,      idacmag,  1, 6);  // Low-side power switch
DEF_REG_BIT(imag_sel,         idacmag,  4, 0);  // IDAC magnitude selection
DEF_REG_BIT(i2mux,            idacmux,  4, 4);  // IDAC2 output channel selection
DEF_REG_BIT(i1mux,            idacmux,  4, 0);  // IDAC1 output channel selection

DEF_REG_BIT(vb_level,         vbias,    1, 7);  // VBIAS level selection
DEF_REG_BIT(vb_ainc,          vbias,    1, 6);  // AINCOM VBIAS selection
DEF_REG_BIT(vb_ain5,          vbias,    1, 5);  // AIN5 VBIAS selection
DEF_REG_BIT(vb_ain4,          vbias,    1, 4);  // AIN4 VBIAS selection
DEF_REG_BIT(vb_ain3,          vbias,    1, 3);  // AIN3 VBIAS selection
DEF_REG_BIT(vb_ain2,          vbias,    1, 2);  // AIN2 VBIAS selection
DEF_REG_BIT(vb_ain1,          vbias,    1, 1);  // AIN1 VBIAS selection
DEF_REG_BIT(vb_ain0,          vbias,    1, 0);  // AIN0 VBIAS selection

DEF_REG_BIT(sys_mon_conf,     sys,      3, 5);  // System monitor configuration
DEF_REG_BIT(cal_samp_size,    sys,      2, 3);  // Calibration sample size selection
DEF_REG_BIT(spi_timeout,      sys,      1, 2);  // SPI timeout enable
DEF_REG_BIT(crc_en,           sys,      1, 1);  // CRC enable
DEF_REG_BIT(status_byte_en,   sys,      1, 0);  // STATUS byte enable

DEF_REG_BIT(offset_cal0,      ofcal0,   8, 0);  // OFC[7:0]   Offset Calibration Register
DEF_REG_BIT(offset_cal1,      ofcal1,   8, 0);  // OFC[15:8]  Offset Calibration Register
DEF_REG_BIT(offset_cal2,      ofcal2,   8, 0);  // OFC[23:16] Offset Calibration Register

DEF_REG_BIT(gain_cal0,        fscal0,   8, 0);  // FSC[7:0]   Gain Calibration Register
DEF_REG_BIT(gain_cal1,        fscal1,   8, 0);  // FSC[15:8]  Gain Calibration Register
DEF_REG_BIT(gain_cal2,        fscal2,   8, 0);  // FSC[23:16] Gain Calibration Register

DEF_REG_BIT(gpio_dir,         gpiodat,  4, 4);  // GPIO direction
DEF_REG_BIT(gpio_data,        gpiodat,  4, 0);  // GPIO data

DEF_REG_BIT(gpio_con,         gpiocon,  4, 0);  // GPIO pin configuration

#undef DEF_REG_BIT

typedef enum {
  ads124s_cmd_nop     = 0x00,
  ads124s_cmd_wakeup  = 0x02,
  ads124s_cmd_pwrdwn  = 0x04,
  ads124s_cmd_reset   = 0x06,
  ads124s_cmd_start   = 0x08,
  ads124s_cmd_stop    = 0x0a,
  ads124s_cmd_syocal  = 0x16,
  ads124s_cmd_sygcal  = 0x17,
  ads124s_cmd_sfocal  = 0x19,
  ads124s_cmd_rdata   = 0x12,
  ads124s_cmd_rreg    = 0x20,
  ads124s_cmd_wreg    = 0x40,
} ads124s_cmd;

typedef enum {
  ads124s_chan_ain0   = 0x00,
  ads124s_chan_ain1   = 0x01,
  ads124s_chan_ain2   = 0x02,
  ads124s_chan_ain3   = 0x03,
  ads124s_chan_ain4   = 0x04,
  ads124s_chan_ain5   = 0x05,
  ads124s_chan_aincom = 0x0c,
  /* ADS124S08 only channels */
  ads124s_chan_ain6   = 0x06,
  ads124s_chan_ain7   = 0x07,
  ads124s_chan_ain8   = 0x08,
  ads124s_chan_ain9   = 0x09,
  ads124s_chan_ain10  = 0x0a,
  ads124s_chan_ain11  = 0x0b,
} ads124s_channel;

typedef enum {
  ads124s_conv_delay_x14    = 0x00,   // n * tMOD
  ads124s_conv_delay_x25    = 0x01,
  ads124s_conv_delay_x64    = 0x02,
  ads124s_conv_delay_x256   = 0x03,
  ads124s_conv_delay_x1024  = 0x04,
  ads124s_conv_delay_x2048  = 0x05,
  ads124s_conv_delay_x4096  = 0x06,
  ads124s_conv_delay_x1     = 0x07,
} ads124s_conv_delay;

typedef enum {
  ads124s_pga_gain_x1   = 0x00,
  ads124s_pga_gain_x2   = 0x01,
  ads124s_pga_gain_x4   = 0x02,
  ads124s_pga_gain_x8   = 0x03,
  ads124s_pga_gain_x16  = 0x04,
  ads124s_pga_gain_x32  = 0x05,
  ads124s_pga_gain_x64  = 0x06,
  ads124s_pga_gain_x128 = 0x07,
} ads124s_pga_gain;

typedef enum {
  ads124s_datarate_x2_5   = 0x00,
  ads124s_datarate_x5     = 0x01,
  ads124s_datarate_x10    = 0x02,
  ads124s_datarate_x16_6  = 0x03,
  ads124s_datarate_x20    = 0x04,
  ads124s_datarate_x50    = 0x05,
  ads124s_datarate_x60    = 0x06,
  ads124s_datarate_x100   = 0x07,
  ads124s_datarate_x200   = 0x08,
  ads124s_datarate_x400   = 0x09,
  ads124s_datarate_x800   = 0x0a,
  ads124s_datarate_x1000  = 0x0b,
  ads124s_datarate_x2000  = 0x0c,
  ads124s_datarate_x4000  = 0x0d,
} ads124s_datarate;

typedef enum {
  ads124s_refmon_disabled = 0x00,
  ads124s_refmon_l0       = 0x01,
  ads124s_refmon_l0_l1    = 0x02,
  ads124s_refmon_l0_10m   = 0x03,
} ads124s_refmon;

typedef enum {
  ads124s_refsel_p0n0     = 0x00,
  ads124s_refsel_p1n1     = 0x01,
  ads124s_refsel_internal = 0x02,
} ads124s_refsel;

typedef enum {
  ads124s_refcon_off        = 0x00,
  ads124s_refcon_pwrdwn     = 0x01,
  ads124s_refcon_always_on  = 0x02,
} ads124s_refcon;

typedef enum {
  ads124s_psw_open  = 0,
  ads124s_psw_close = 1,
} ads124s_psw;

typedef enum {
  ads124s_imag_off    = 0x00,
  ads124s_imag_10mu   = 0x01,
  ads124s_imag_50mu   = 0x02,
  ads124s_imag_100mu  = 0x03,
  ads124s_imag_250mu  = 0x04,
  ads124s_imag_500mu  = 0x05,
  ads124s_imag_750mu  = 0x06,
  ads124s_imag_1000mu = 0x07,
  ads124s_imag_1500mu = 0x08,
  ads124s_imag_2000mu = 0x09,
} ads124s_imag;

typedef enum {
  ads124s_vbias_div_2 = 0,
  ads124s_vbias_div_12 = 1,
} ads124s_vbias;

typedef enum {
  ads124s_sysmon_disabled       = 0x00,
  ads124s_sysmon_pga_short      = 0x01,
  ads124s_sysmon_temp_sensor    = 0x02,
  ads124s_sysmon_avdd_avss_4    = 0x03,
  ads124s_sysmon_dvdd_4         = 0x04,
  ads124s_sysmon_burnout_0_2mu  = 0x05,
  ads124s_sysmon_burnout_1mu    = 0x06,
  ads124s_sysmon_burnout_10mu   = 0x07,
} ads124s_sysmon;

typedef enum {
  ads124s_cal_sample_x1   = 0x00,
  ads124s_cal_sample_x4   = 0x01,
  ads124s_cal_sample_x8   = 0x02,
  ads124s_cal_sample_x16  = 0x03,
} ads124s_cal_sample;

typedef enum {
  ads124s_gpio_dir_out  = 0,
  ads124s_gpio_dir_in   = 1,
} ads124s_gpio_dir_conf;

typedef enum {
  ads124s_gpio_conf_ain   = 0,
  ads124s_gpio_conf_gpio  = 1,
} ads124s_gpio_conf;


static INLINE void      ads124s_set_value(ads124s_register_bit field, uint8_t value);
static INLINE uint8_t   ads124s_get_value(ads124s_register_bit field);
static INLINE void      ads124s_send_cmd(uint8_t cmd);
static INLINE void      ads124s_update_matching_reg(ads124s_register_bit field);

void ads124s_reset();
void ads124s_performSelfGainCalibration();
void ads124s_performSelfOffsetCalibration();
void ads124s_performSystemOffsetCalibration();
void ads124s_read_conv_data(uint32_t *conv_data);
void ads124s_update_reg(ads124s_register* reg);
void ads124s_read_reg(ads124s_register* reg, uint8_t num, uint8_t* buf)


/** implementation starts here */
static INLINE void
ads124s_set_value(ads124s_register_bit field, uint8_t value)
{
  /* convert the numbers of bits in a mask with matching length */
  const uint8_t mask = ((uint8_t)1 << field.bits) - 1;
  /* clear affected bits */
  field.reg->value &= ~(mask << field.offset);
  /* set affected bits */
  field.reg->value |= ((value & mask) << field.offset);
}

static INLINE uint8_t
ads124s_get_value(ads124s_register_bit field)
{
  /* convert the numbers of bits in a mask with matching length */
  const uint8_t mask = ((uint8_t)1 << field.bits) - 1;
  return (field.reg->value >> field.offset) & mask;
}

static INLINE void
ads124s_send_cmd(uint8_t cmd)
{
  HAL_SPI_Transmit(ads124s_dev, lit2addr(cmd), 1, ads124s_spi_timeout);
}

static INLINE void
ads124s_update_matching_reg(ads124s_register_bit field)
{
  ads124s_update_reg(field.reg);
}

#ifdef  __cplusplus
}
#endif
#endif /* __ADS124S0X_H */
