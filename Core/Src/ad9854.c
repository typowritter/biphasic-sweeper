#include "ad9854.h"
#include "delay.h"

/* singleton */
ad9854_registers ad9854_regs = {
  .par1           = {.s_addr = 0x00, .p_addr = 0x00, .value = 0x00, .size = 2 },
  .par2           = {.s_addr = 0x01, .p_addr = 0x02, .value = 0x00, .size = 2 },
  .ftw1           = {.s_addr = 0x02, .p_addr = 0x04, .value = 0x00, .size = 6 },
  .ftw2           = {.s_addr = 0x03, .p_addr = 0x0a, .value = 0x00, .size = 6 },
  .dfw            = {.s_addr = 0x04, .p_addr = 0x10, .value = 0x00, .size = 6 },
  .update_clk     = {.s_addr = 0x05, .p_addr = 0x16, .value = 0x40, .size = 4 },
  .ramp_rate_clk  = {.s_addr = 0x06, .p_addr = 0x1a, .value = 0x00, .size = 3 },
  .cr             = {.s_addr = 0x07, .p_addr = 0x1d, .value = 0x10640120, .size = 4 },
  .osk_i_mult     = {.s_addr = 0x08, .p_addr = 0x21, .value = 0x00, .size = 2 },
  .osk_q_mult     = {.s_addr = 0x09, .p_addr = 0x23, .value = 0x00, .size = 2 },
  .osk_ramp_rate  = {.s_addr = 0x0a, .p_addr = 0x25, .value = 0x80, .size = 1 },
  .qdac           = {.s_addr = 0x0b, .p_addr = 0x26, .value = 0x00, .size = 2 },
};

void ad9854_init()
{
  gpio_set_low(ad9854_pin_udclk);
#if USE_PARALLEL
  gpio_set_high(ad9854_pin_wr);
  // gpio_set_high(ad9854_pin_rd);
#endif
  ad9854_reset();

#if !(USE_PARALLEL)
  ad9854_set_bits(ad9854_sdo_cr, 1);
#endif
  ad9854_set_bits(ad9854_updclk, ad9854_updclk_external);
  ad9854_set_bits(ad9854_mode, ad9854_mode_single);
  ad9854_set_bits(ad9854_invsinc_byp, 1);
  ad9854_set_bits(ad9854_osk_en, 1);
  ad9854_update_reg(&ad9854_regs.cr);

  delay_ms(10);
}

void ad9854_reset()
{
  gpio_set_high(ad9854_pin_rst);
  delay_us(1);  /* 10 sysclk */
  gpio_set_low(ad9854_pin_rst);
}

void freq_convert(uint64_t freq)
{
  /* FTW = (freq × 2^N)/SYSCLK */
  ad9854_regs.ftw1.value = freq * (((uint64_t)1<<48)/ad9854_sysclk);
  ad9854_update_reg(&ad9854_regs.ftw1);
}

void amp_convert(uint16_t amp)
{
  /* 12-bit, 0~4095 */
  ad9854_regs.osk_i_mult.value = amp;
  ad9854_regs.osk_q_mult.value = amp;
  ad9854_update_reg(&ad9854_regs.osk_i_mult);
  ad9854_update_reg(&ad9854_regs.osk_q_mult);
}

#if USE_PARALLEL
uint8_t ad9854_read_byte(uint8_t addr)
{
  todo();
  return 0;
}

void ad9854_write_byte(uint8_t addr, uint8_t data)
{
  gpio_set_group(ad9854_par_addr, addr);
  gpio_set_group(ad9854_par_data, data);
  gpio_set_low(ad9854_pin_wr);
  delay_us(1);  // tWRLOW = 2.5ns
  gpio_set_high(ad9854_pin_wr);
  delay_us(1);  // tWRHOGH = 7ns
}

uint64_t ad9854_read_parallel(ad9854_register* reg)
{
  todo();
  return 0;
}

void ad9854_write_parallel(ad9854_register* reg, uint64_t value)
{
  uint8_t p_addr = reg->p_addr;
  for (uint8_t i = 0; i < reg->size; i++)
  {
    ad9854_write_byte(p_addr,
      (reg->value) >> 8 * (reg->size - i - 1));

    p_addr++;
  }

  gpio_set_high(ad9854_pin_udclk);
  delay_us(1);
  gpio_set_low(ad9854_pin_udclk);
}

#else
uint64_t ad9854_read_serial(ad9854_register* reg)
{
  todo();
  return 0;
}

void ad9854_write_serial(ad9854_register* reg, uint64_t value)
{
  uint8_t tx_buffer[7];
  tx_buffer[0] = reg->s_addr | (0<<7);  /* instr write */

  for (int i = 1; i <= reg->size; i++)
  {
    tx_buffer[i] = (value >> ((reg->size - i) * 8)) & 0xff;
  }

  ad9854_select();

  HAL_SPI_Transmit(&ad9854_dev, tx_buffer, 1 + reg->size, ad9854_spi_timeout);
  ad9854_unselect();

  gpio_set_high(ad9854_pin_udclk);
  delay_us(1);
  gpio_set_low(ad9854_pin_udclk);
}
#endif