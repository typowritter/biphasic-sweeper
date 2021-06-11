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
  gpio_set_high(ad9854_pin_wr);
  // gpio_set_high(ad9854_pin_rd);

  ad9854_reset();

  ad9854_set_bits(ad9854_updclk, ad9854_updclk_external);
  ad9854_set_bits(ad9854_mode, ad9854_mode_single);
  ad9854_update_reg(&ad9854_regs.cr);

  delay_ms(10);
}

void ad9854_reset()
{
  gpio_set_high(ad9854_pin_rst);
  delay_us(1);  /* 10 sysclk */
  gpio_set_low(ad9854_pin_rst);
}

uint8_t ad9854_read_byte(uint8_t addr)
{
  undefined();
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
  undefined();
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
