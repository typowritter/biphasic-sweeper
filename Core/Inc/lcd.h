/**
  ******************************************************************************
  * @file    lcd.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-16
  * @brief   driver for onboard 5-inch 800x480 LCD
  *          use sdram as video memory
  ******************************************************************************
  */

#ifndef __LCD_H
#define __LCD_H

#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

#include "ltdc.h"
#include "dma2d.h"
#include "sdram.h"
#include "fonts.h"
#include "utils.h"

/* BEGIN project specific setups
 * keep those synchronized with global settings */
#define OUTPUT_COLOR_MODE  DMA2D_OUTPUT_RGB565
#define MAX_LAYERS         1 /* one layer */
#define LCD_WIDTH          1024
#define LCD_HEIGHT         600

/* we use RGB888 internally, DMA2D will do the conversion on output */
typedef uint32_t color_t;     /* RGB888 for use */
typedef uint16_t o_color_t;   /* RGB565 for output */
typedef uint32_t vmem_addr_t; /* video memory address */

/* END project specific setups */

#define VRAM_PLANE       0
#define FONTCACHE_PLANE  1
#define MISC_PLANE       2

#define pos2addr(x, y) \
  (SDRAM_BANK_ADDR + sizeof(o_color_t) * ((LCD_WIDTH) * (y) + (x)))

#define get_plane_addr(plane) \
  (SDRAM_BANK_ADDR + (plane) * sizeof(color_t) * LCD_WIDTH * LCD_HEIGHT)

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


/* global struct of the lcd */
extern lcd_configs_t lcd;

static INLINE void lcd_init();
static INLINE void lcd_clear();
static INLINE void draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t t);
static INLINE void draw_vline(uint16_t x, uint16_t y, uint16_t len, uint16_t t);
static INLINE void draw_pixel(uint16_t x, uint16_t y, color_t color);
static INLINE void draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t t);
static INLINE uint16_t to_rgb565(color_t rgb888);

/**
 * fill a rectangular region
 *
 * @param x,y     -- start position
 * @param w,h     -- width & height
 * @param color   -- color to fill
 */
void fill_region(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color_t color);

/**
 * draw a RGB88 bitmap
 *
 * @param x,y     -- start position
 * @param w,h     -- width & height
 * @param bitmap  -- address of RGB888 bitmap
 */
void draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, vmem_addr_t bitmap);

/**
 * display a character on (x,y)
 */
void disp_char(uint16_t x, uint16_t y, char ch);

/**
 * display a string on (x,y)
 */
void disp_string(uint16_t x, uint16_t y, char *str);

/**
 * draw an arbitrary skewed line, using Bresenham algorithm
 */
void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/** implementation starts here */
static INLINE void
lcd_init()
{
  sdram_init(); /* use sdram as video memory */
  lcd_clear();
}

static INLINE void
lcd_clear()
{
  fill_region(0, 0, LCD_WIDTH, LCD_HEIGHT, lcd.back_color);
}

/**
 * draw a horizontal line
 *
 * @param x,y  -- start position
 * @param len  -- length of the line
 * @param t    -- thickness of the line
 */
static INLINE void
draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t t)
{
  fill_region(x, y, len, t, lcd.fore_color);
}

/**
 * draw a vertical line
 *
 * @param x,y  -- start position
 * @param len  -- length of the line
 * @param t    -- thickness of the line
 */
static INLINE void
draw_vline(uint16_t x, uint16_t y, uint16_t len, uint16_t t)
{
  fill_region(x, y, t, len, lcd.fore_color);
}

static INLINE void
draw_pixel(uint16_t x, uint16_t y, color_t color)
{
  *(o_color_t *) pos2addr(x, y) = to_rgb565(color);
}

/**
 * draw a rectangle
 *
 * @param x,y  -- start position
 * @param w,h  -- size of the rectangle
 * @param t    -- thickness of the edge
 */
static INLINE void
draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t t)
{
  draw_hline(x, y, w, t);
  draw_vline(x, y, h, t);
  draw_hline(x, y+h, w+t, t);
  draw_vline(x+w, y, h+t, t);
}


static INLINE uint16_t
to_rgb565(color_t rgb888)
{
  return ((rgb888 & 0xf80000) >> 8)
       + ((rgb888 & 0xfc00) >> 5)
       + ((rgb888 & 0xf8) >> 3);
}

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __LCD_H */
