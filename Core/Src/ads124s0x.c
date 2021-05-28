#include "ads124s0x.h"
#include "delay.h"

/* singleton */
ads124s_registers ads124s_regs = {
  .id       = {.addr = 0x00, .value = 0x00, .is_volatile = false },
  .status   = {.addr = 0x01, .value = 0x80, .is_volatile = true },
  .inpmux   = {.addr = 0x02, .value = 0x01, .is_volatile = false },
  .pga      = {.addr = 0x03, .value = 0x00, .is_volatile = false },
  .datarate = {.addr = 0x04, .value = 0x14, .is_volatile = false },
  .ref      = {.addr = 0x05, .value = 0x10, .is_volatile = false },
  .idacmag  = {.addr = 0x06, .value = 0x00, .is_volatile = false },
  .idacmux  = {.addr = 0x07, .value = 0xff, .is_volatile = false },
  .vbias    = {.addr = 0x08, .value = 0x00, .is_volatile = false },
  .sys      = {.addr = 0x09, .value = 0x10, .is_volatile = false },
  .ofcal0   = {.addr = 0x0a, .value = 0x00, .is_volatile = true },
  .ofcal1   = {.addr = 0x0b, .value = 0x00, .is_volatile = true },
  .ofcal2   = {.addr = 0x0c, .value = 0x00, .is_volatile = true },
  .fscal0   = {.addr = 0x0d, .value = 0x00, .is_volatile = true },
  .fscal1   = {.addr = 0x0e, .value = 0x00, .is_volatile = true },
  .fscal2   = {.addr = 0x0f, .value = 0x40, .is_volatile = true },
  .gpiodat  = {.addr = 0x10, .value = 0x00, .is_volatile = false },
  .gpiocon  = {.addr = 0x11, .value = 0x00, .is_volatile = false },
};

static uint8_t tx_buffer[5];  /* transmit up to 3 regs in one run */
static uint8_t rx_buffer[5];

void ads124s_reset()
{
  ads124s_send_cmd(ads124s_cmd_reset);
  delay_ms(1);  /* td(RSSC) in internal clk */
}

void ads124s_read_regs(ads124s_register* reg, uint8_t num, uint8_t* data)
{
  tx_buffer[0] = ads124s_cmd_rreg | reg->addr;
  tx_buffer[1] = num - 1;         /* opcode 1 reg */
  tx_buffer[2] = ads124s_cmd_nop; /* dummy */

  ads124s_select();

  HAL_SPI_TransmitReceive(&ads124s_dev, tx_buffer, rx_buffer,
    num+2, ads124s_spi_timeout);

  for (int i = 0; i < num; i++)
  {
    data[i] = rx_buffer[i+2];
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

void ads124s_performSystemOffsetCalibration()
{
  ads124s_send_cmd(ads124s_cmd_syocal);
}