/**
  ******************************************************************************
  * @file    gt9157.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-12
  * @brief   driver for onboard 5-inch 800x480 LCD
  *          with the touch controller GT9157
  ******************************************************************************
  */

#include "gt9157.h"
#include "delay.h"

void gt9157_init()
{
    /* rst->HIGH, int->LOW, I2C Slave Addr configured as 0xBA */
    gpio_set_high(gt9157_pin_rst);
    delay_ms(10);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GTP_INT_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GTP_INT_GPIO_Port, &GPIO_InitStructure);
}