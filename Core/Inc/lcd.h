/**
  ******************************************************************************
  * @file    lcd.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-16
  * @brief   driver for onboard 5-inch 800x480 LCD
  ******************************************************************************
  */

#ifndef __LCD_H
#define __LCD_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "ltdc.h"
#include "dma2d.h"
#include "fonts.h"

/* BEGIN project specific setups
 * keep those synchronized with global settings */
#define DMA2D_COLOR_MODE   DMA2D_OUTPUT_RGB565
#define MAX_LAYERS         1 /* one layer */

/* we use RGB888 internally, DMA2D will do the conversion on output */
typedef uint32_t color_t;
typedef uint32_t vmem_addr_t; /* video memory address */

/* END project specific setups */

enum rgb888_enum {
  aqua       = 0x00ffff,
  black      = 0x000000,
  blue       = 0x0000ff,
  cyan       = 0x00ffff,
  darkcyan   = 0x007d7b,
  darkgreen  = 0x007d00,
  darkgrey   = 0x7b7d7b,
  fuchsia    = 0xff00ff,
  gray1      = 0x848284,
  gray2      = 0x424142,
  gray3      = 0x212021,
  green      = 0x00ff00,
  lightgreen = 0xadff29,
  lightgrey  = 0xc5c2c5,
  lime       = 0x00ff00,
  magenta    = 0xff00ff,
  maroon     = 0x7b0000,
  navy       = 0x00007b,
  olive      = 0x848200,
  orange     = 0xffa600,
  purple     = 0x7b007b,
  red        = 0xff0000,
  silver     = 0xc5c2c5,
  teal       = 0x008284,
  white      = 0xffffff,
  yellow     = 0xffff00,
};

typedef struct
{
  uint16_t width;
  uint16_t height;
  uint8_t  layer;
  sFONT*   font;
  color_t  fore_color;
  color_t  back_color;
} lcd_configs_t;

/**
 * initialize lcd display
 */
void lcd_init();


#ifdef  __cplusplus
}
#endif
#endif /* __LCD_H */
