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

static uint8_t tx_buffer[5];  /* transmit up to 3 bytes in one run */
static uint8_t rx_buffer[5];  /* if enable STATUS or CRC byte, must be increased */

void ads124s_init()
{
  ads124s_reset();
  delay_ms(2);  /* td(RSSC) = 4096 * tCLK */
  ads124s_set_value(ads124s_fl_por, 0);
  ads124s_update_matching_reg(ads124s_fl_por);
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

void ads124s_read_conv_data(uint32_t *conv_data)
{
  undefined();
}