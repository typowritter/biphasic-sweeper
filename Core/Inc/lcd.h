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

void lcd_init();


#ifdef  __cplusplus
}
#endif
#endif /* __LCD_H */
