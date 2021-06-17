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

#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"

static vmem_addr_t char2bitmap(char ch, uint8_t plane);

/* singleton */
lcd_configs_t lcd = {
  .width  = 800,
  .height = 480,
  .layer  = 0,
  .font   = &Font16x24,
  .fore_color = black,
  .back_color = white,
};

/**
 * fill a rectangular region
 *
 * @param x,y     -- start position
 * @param w,h     -- width & height
 * @param color   -- color to fill
 */
void fill_region(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color_t color)
{
  vmem_addr_t dst = pos2addr(x, y);

  hdma2d.Instance          = DMA2D;
  hdma2d.Init.Mode         = DMA2D_R2M;
  hdma2d.Init.ColorMode    = OUTPUT_COLOR_MODE;
  hdma2d.Init.OutputOffset = lcd.width - w;

  if (HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if (HAL_DMA2D_ConfigLayer(&hdma2d, lcd.layer) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, color, dst, w, h) == HAL_OK)
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

/**
 * draw a RGB88 bitmap
 *
 * @param x,y     -- start position
 * @param w,h     -- width & height
 * @param bitmap  -- address of RGB888 bitmap
 */
void draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, vmem_addr_t bitmap)
{
  vmem_addr_t dst = pos2addr(x, y);

  hdma2d.Instance          = DMA2D;
  hdma2d.Init.Mode         = DMA2D_M2M_PFC; /* perform the conversion to RGB565 */
  hdma2d.Init.ColorMode    = OUTPUT_COLOR_MODE;
  hdma2d.Init.OutputOffset = lcd.width - w;

  if (HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if (HAL_DMA2D_ConfigLayer(&hdma2d, lcd.layer) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, bitmap, dst, w, h) == HAL_OK)
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

/**
 * display a character on (x,y)
 */
void disp_char(uint16_t x, uint16_t y, char ch)
{
  static vmem_addr_t cached_font[127 - ' '];

  uint8_t index = ch - ' ';

  /* only cache the most used 16x24 font */
  if (lcd.font->width == 16)
  {
    if (!cached_font[index])
    {
      cached_font[index] = char2bitmap(ch, FONTCACHE_PLANE);
    }
    draw_bitmap(x, y, lcd.font->width, lcd.font->height, cached_font[index]);
  }
  else /* font of other sizes */
  {
    vmem_addr_t font_addr = char2bitmap(ch, MISC_PLANE);
    draw_bitmap(x, y, lcd.font->width, lcd.font->height, font_addr);
  }
}

/**
 * draw an arbitrary skewed line, using Bresenham algorithm
 */
void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  uint16_t us;
  uint16_t cur_x, cur_y;

  int32_t err_x = 0, err_y = 0, delta_x, delta_y, distance;
  int32_t inc_x, inc_y;

  delta_x = x2 - x1;
  delta_y = y2 - y1;

  cur_x = x1;
  cur_y = y1;

  if (delta_x > 0)
  {
    inc_x = 1;
  }
  else if (delta_x == 0)
  {
    inc_x = 0;
  }
  else
  {
    inc_x = -1;
    delta_x = - delta_x;
  }


  if (delta_y > 0)
  {
    inc_y = 1;
  }
  else if (delta_y == 0)
  {
    inc_y = 0;
  }
  else
  {
    inc_y = -1;
    delta_y = - delta_y;
  }

  if (delta_x > delta_y)
    distance = delta_x;
  else
    distance = delta_y;


  for (us = 0; us <= distance + 1; us++)
  {
    draw_pixel(cur_x, cur_y, lcd.fore_color);

    err_x += delta_x ;
    err_y += delta_y ;

    if (err_x > distance)
    {
      err_x -= distance;
      cur_x += inc_x;
    }

    if (err_y > distance)
    {
      err_y -= distance;
      cur_y += inc_y;
    }

  }
}

/** ---------------- static definitions ---------------- */

/**
 * cache a character, returning the start address in the cache plane
 */
static vmem_addr_t char2bitmap(char ch, uint8_t plane)
{
  /* points per character */
  uint16_t point_cnt = lcd.font->height * lcd.font->width;

  /* bytes per character */
  uint8_t byte_cnt = point_cnt / 8;

  /* index in the font table (offset = 32) */
  uint16_t index = ch - ' ';

  /* start position in the font table */
  uint8_t *table = (uint8_t *)&(lcd.font->table)[index * byte_cnt];

  /* start position in the video memory */
  vmem_addr_t m_offset = get_plane_addr(plane)
                       + sizeof(color_t) * point_cnt * index;

  sdram_data_t *bitmap = (sdram_data_t *)m_offset;

  for (uint8_t byte = 0; byte < byte_cnt; byte++)
  {
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (table[byte] & (0x80>>bit))
      {
        *bitmap = lcd.fore_color;
      }
      else
      {
        *bitmap = lcd.back_color;
      }
      bitmap += 1;
    }
  }

  return m_offset;
}
