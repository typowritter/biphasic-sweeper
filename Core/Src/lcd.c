/**
  ******************************************************************************
  * @file    lcd.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-16
  * @brief   driver for onboard 5-inch 800x480 LCD
  *          use sdram as video memory
  ******************************************************************************
  */

#include "lcd.h"
#include "sdram.h"

/* singleton */
static lcd_configs_t lcd = {
  .width  = 800,
  .height = 480,
  .layer  = 0,
  .font   = &Font8x16,
  .fore_color = black,
  .back_color = white,
};

static void fill_region(uint16_t x, uint16_t y, uint16_t len_x, uint16_t len_y,
                        color_t color);

void lcd_clear()
{
  fill_region(0, 0, lcd.width, lcd.height, lcd.back_color);
}

void lcd_init()
{
  sdram_init(); /* use sdram as video memory */
  lcd_clear();
}

static void
fill_region(uint16_t x, uint16_t y, uint16_t len_x, uint16_t len_y, color_t color)
{
  uint32_t addr_dst = SDRAM_BANK_ADDR
                    + sizeof(color_t) * (lcd.width * y + x);

  hdma2d.Instance          = DMA2D;
  hdma2d.Init.Mode         = DMA2D_R2M;
  hdma2d.Init.ColorMode    = DMA2D_COLOR_MODE;
  hdma2d.Init.OutputOffset = lcd.width - len_x;

  if (HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if (HAL_DMA2D_ConfigLayer(&hdma2d, lcd.layer) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, color, addr_dst, len_x, len_y) == HAL_OK)
      {
        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
        goto _success;
      }
    }
  }

  Error_Handler();

  _success:
  return;
}
