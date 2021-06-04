#include "ad9854.h"
#include "delay.h"

/* singleton */
ads124s_registers ads124s_regs = {
  .par1           = {.addr = 0x00, .value = 0x00, .size = 2 },
  .par2           = {.addr = 0x01, .value = 0x00, .size = 2 },
  .ftw1           = {.addr = 0x02, .value = 0x00, .size = 6 },
  .ftw2           = {.addr = 0x03, .value = 0x00, .size = 6 },
  .dfw            = {.addr = 0x04, .value = 0x00, .size = 6 },
  .update_clk     = {.addr = 0x05, .value = 0x40, .size = 4 },
  .ramp_rate_clk  = {.addr = 0x06, .value = 0x00, .size = 3 },
  .cr             = {.addr = 0x07, .value = 0x10640120, .size = 4 },
  .osk_i_mult     = {.addr = 0x08, .value = 0x00, .size = 2 },
  .osk_q_mult     = {.addr = 0x09, .value = 0x00, .size = 2 },
  .osk_ramp_rate  = {.addr = 0x0a, .value = 0x80, .size = 1 },
  .qdac           = {.addr = 0x0b, .value = 0x00, .size = 2 },
};

static uint8_t tx_buffer[5];  /* transmit up to 3 bytes in one run */
static uint8_t rx_buffer[5];  /* if enable STATUS or CRC byte, must be increased */

void ads124s_test()
{
  ads124s_read_reg(&ads124s_regs.status);
}

void ads124s_init()
{
  ads124s_reset();
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
