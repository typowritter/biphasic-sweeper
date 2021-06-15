/**
  ******************************************************************************
  * @file    lcd.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-06-16
  * @brief   driver for onboard 5-inch 800x480 LCD
  ******************************************************************************
  */

#include "lcd.h"
#include "main.h"

void lcd_init()
{
  hdma2d.Init.Mode         = DMA2D_R2M;
  hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;

  hdma2d.Instance = DMA2D;

  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 0) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, 0xffff, 0xD0000000, 800, 480) == HAL_OK)
      {
        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
        return;
      }
    }
  }
  Error_Handler();
}