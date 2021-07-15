#include "ads124s0x.h"
#include "delay.h"

/* singleton */
ads124s_registers ads124s_regs = {
  .id       = {.addr = 0x00, .value = 0x00 },
  .status   = {.addr = 0x01, .value = 0x80 },
  .inpmux   = {.addr = 0x02, .value = 0x01 },
  .pga      = {.addr = 0x03, .value = 0x00 },
  .datarate = {.addr = 0x04, .value = 0x14 },
  .ref      = {.addr = 0x05, .value = 0x10 },
  .idacmag  = {.addr = 0x06, .value = 0x00 },
  .idacmux  = {.addr = 0x07, .value = 0xff },
  .vbias    = {.addr = 0x08, .value = 0x00 },
  .sys      = {.addr = 0x09, .value = 0x10 },
  .ofcal0   = {.addr = 0x0a, .value = 0x00 },
  .ofcal1   = {.addr = 0x0b, .value = 0x00 },
  .ofcal2   = {.addr = 0x0c, .value = 0x00 },
  .fscal0   = {.addr = 0x0d, .value = 0x00 },
  .fscal1   = {.addr = 0x0e, .value = 0x00 },
  .fscal2   = {.addr = 0x0f, .value = 0x40 },
  .gpiodat  = {.addr = 0x10, .value = 0x00 },
  .gpiocon  = {.addr = 0x11, .value = 0x00 },
};

static uint8_t tx_buffer[6];  /* transmit up to 5 bytes in one run */
static uint8_t rx_buffer[6];  /* minimum size required by STATUS or CRC byte */

void ads124s_init()
{
  ads124s_reset();
  delay_ms(2);  /* td(RSSC) = 4096 * tCLK */
  ads124s_set_value(ads124s_fl_por, 0);
  ads124s_update_matching_reg(ads124s_fl_por);
  ads124s_set_value(ads124s_conv_mode, ads124s_mode_cont);
  ads124s_update_matching_reg(ads124s_conv_mode);
  ads124s_set_value(ads124s_status_byte_en, 1);
  ads124s_update_matching_reg(ads124s_status_byte_en);
}

void ads124s_reset()
{
  gpio_set_low(ads124s_pin_rst);
  delay_us(50);  /* tw(RSL) = 4 * tCLK */
  gpio_set_high(ads124s_pin_rst);
}

void ads124s_read_regs(ads124s_register* reg, uint8_t num)
{
  tx_buffer[0] = ads124s_cmd_rreg | reg->addr;
  tx_buffer[1] = num - 1;         /* opcode 1 reg */
  tx_buffer[2] = ads124s_cmd_nop; /* dummy */
  tx_buffer[3] = ads124s_cmd_nop; /* dummy */
  tx_buffer[4] = ads124s_cmd_nop; /* dummy */

  ads124s_select();

  HAL_SPI_TransmitReceive(&ads124s_dev, tx_buffer, rx_buffer,
    num+2, ads124s_spi_timeout);

  for (int i = 0; i < num; i++)
  {
    (reg++)->value = rx_buffer[i+2];
  }

  ads124s_unselect();
}

void ads124s_write_regs(ads124s_register* reg, uint8_t num, uint8_t* data)
{
  tx_buffer[0] = ads124s_cmd_wreg | reg->addr;
  tx_buffer[1] = num - 1;         /* opcode 1 reg */

  for (int i = 0; i < num; i++)
  {
    tx_buffer[i+2] = data[i];
    (reg++)->value = data[i];
  }

  ads124s_select();
  HAL_SPI_Transmit(&ads124s_dev, tx_buffer, num+2, ads124s_spi_timeout);
  ads124s_unselect();
}

ads124s_conv_result_t ads124s_read_conv_data()
{
  tx_buffer[0] = ads124s_cmd_rdata ;
  tx_buffer[1] = ads124s_cmd_nop; /* STATUS */
  tx_buffer[2] = ads124s_cmd_nop; /* Data 1 */
  tx_buffer[3] = ads124s_cmd_nop; /* Data 2 */
  tx_buffer[4] = ads124s_cmd_nop; /* Data 3 */

  ads124s_select();

  HAL_SPI_TransmitReceive(&ads124s_dev, tx_buffer, rx_buffer,
    5, ads124s_spi_timeout);

  ads124s_conv_result_t res;
  if (ads124s_get_value(ads124s_status_byte_en))
  {
    res.status = rx_buffer[1];
    res.crc    = rx_buffer[5];  /* garbage if not enabled crc */
    res.data   = rx_buffer[4];
    res.data  |= rx_buffer[3] << 8;
    res.data  |= rx_buffer[2] << 16;
  }
  else
  {
    res.crc    = rx_buffer[4];  /* garbage if not enabled crc */
    res.data   = rx_buffer[3];
    res.data  |= rx_buffer[2] << 8;
    res.data  |= rx_buffer[1] << 16;
  }

  ads124s_unselect();
  return res;
}