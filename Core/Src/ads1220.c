#include "ads1220.h"
#include "delay.h"

/* singleton */
ads1220_register ads1220_regs[] = {
  [0] = {.addr = 0x00, .value = 0x00 },
  [1] = {.addr = 0x01, .value = 0x00 },
  [2] = {.addr = 0x02, .value = 0x00 },
  [3] = {.addr = 0x03, .value = 0x00 },
};

static uint8_t tx_buffer[5];  /* transmit up to 3 bytes in one run */
static uint8_t rx_buffer[5];  /* if enable STATUS or CRC byte, must be increased */

void ads1220_test()
{
  ads1220_read_reg(&ads1220_regs.status);
}

void ads1220_init()
{
  ads1220_reset();
  delay_ms(2);  /* td(RSSC) = 4096 * tCLK */
  ads1220_set_value(ads1220_fl_por, 0);
  ads1220_update_matching_reg(ads1220_fl_por);
}

void ads1220_reset()
{
  gpio_set_low(ads1220_pin_rst);
  delay_us(50);  /* tw(RSL) = 4 * tCLK */
  gpio_set_high(ads1220_pin_rst);
}

void ads1220_read_regs(ads1220_register* reg, uint8_t num)
{
  tx_buffer[0] = ads1220_cmd_rreg | reg->addr;
  tx_buffer[1] = num - 1;         /* opcode 1 reg */
  tx_buffer[2] = ads1220_cmd_nop; /* dummy */
  tx_buffer[3] = ads1220_cmd_nop; /* dummy */
  tx_buffer[4] = ads1220_cmd_nop; /* dummy */

  ads1220_select();

  HAL_SPI_TransmitReceive(&ads1220_dev, tx_buffer, rx_buffer,
    num+2, ads1220_spi_timeout);

  for (int i = 0; i < num; i++)
  {
    (reg++)->value = rx_buffer[i+2];
  }

  ads1220_unselect();
}

void ads1220_write_regs(ads1220_register* reg, uint8_t num, uint8_t* data)
{
  tx_buffer[0] = ads1220_cmd_wreg | reg->addr;
  tx_buffer[1] = num - 1;         /* opcode 1 reg */

  for (int i = 0; i < num; i++)
  {
    tx_buffer[i+2] = data[i];
    (reg++)->value = data[i];
  }

  ads1220_select();
  HAL_SPI_Transmit(&ads1220_dev, tx_buffer, num+2, ads1220_spi_timeout);
  ads1220_unselect();
}

void ads1220_read_conv_data(uint32_t *conv_data)
{
  undefined();
}