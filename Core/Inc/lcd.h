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

typedef uint16_t color_t;    /* RGB565 */

/* END project specific setups */


enum rgb565_enum {
  aqua       = 0x07ff,
  black      = 0x0000,
  blue       = 0x001f,
  cyan       = 0x07ff,
  darkcyan   = 0x03ef,
  darkgreen  = 0x03e0,
  darkgrey   = 0x7bef,
  fuchsia    = 0xf81f,
  gray1      = 0x8410,
  gray2      = 0x4208,
  gray3      = 0x2104,
  green      = 0x07e0,
  lightgreen = 0xafe5,
  lightgrey  = 0xc618,
  lime       = 0x07e0,
  magenta    = 0xf81f,
  maroon     = 0x7800,
  navy       = 0x000f,
  olive      = 0x8400,
  orange     = 0xfd20,
  purple     = 0x780f,
  red        = 0xf800,
  silver     = 0xc618,
  teal       = 0x0410,
  white      = 0xffff,
  yellow     = 0xffe0,
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
