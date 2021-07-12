/**
  ******************************************************************************
  * @file    gt9157.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-12
  * @brief   driver for onboard 5-inch 800x480 LCD
  *          with the touch controller GT9157
  ******************************************************************************
  */

#ifndef __GT9157_H
#define __GT9157_H

#include "main.h"
#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-const-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

#include "gpio_wrapper.h"
#include "i2c.h"
#include "utils.h"

/* BEGIN project specific setups
 * keep those synchronized with global settings */
#define gt9157_dev          hi2c2
#define gt9157_i2c_timeout  1000
DEF_GPIO(gt9157_pin_rst, GTP_RESET_GPIO_Port, GTP_RESET_Pin);
DEF_GPIO(gt9157_pin_int, GTP_INT_GPIO_Port,   GTP_INT_Pin);

/* END project specific setups */


void gt9157_init();

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __GT9157_H */
