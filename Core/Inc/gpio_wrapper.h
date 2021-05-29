/**
  ******************************************************************************
  * @file    gpio_wrapper.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-05-23
  * @brief   encapsulation of gpio for convenience
  ******************************************************************************
  */

#ifndef __GPIO_WRAPPER_H
#define __GPIO_WRAPPER_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "gpio.h"
#include "utils.h"

typedef struct
{
  GPIO_TypeDef* group;
  uint16_t pin;
} gpio_pin;

/* definition of all used GPIO ports */
#define DEF_GPIO(name, _group, _pin) \
  static const gpio_pin name = {.group = _group, .pin = _pin }

/* basic GPIO functions defined as inline to save call time */
static INLINE void            gpio_set_high(gpio_pin pin);
static INLINE void            gpio_set_low(gpio_pin pin);
static INLINE void            gpio_toggle(gpio_pin pin);
static INLINE void            gpio_set(gpio_pin pin, GPIO_PinState value);
static INLINE GPIO_PinState   gpio_get(gpio_pin pin);


/** implementation starts here */

static INLINE void
gpio_set_high(gpio_pin pin)
{
  HAL_GPIO_WritePin(pin.group, pin.pin, GPIO_PIN_SET);
}

static INLINE void
gpio_set_low(gpio_pin pin)
{
  HAL_GPIO_WritePin(pin.group, pin.pin, GPIO_PIN_RESET);
}

static INLINE void
gpio_toggle(gpio_pin pin)
{
  HAL_GPIO_TogglePin(pin.group, pin.pin);
}

static INLINE void
gpio_set(gpio_pin pin, GPIO_PinState value)
{
  if (value) {
    gpio_set_high(pin);
  } else {
    gpio_set_low(pin);
  }
}

static INLINE GPIO_PinState
gpio_get(gpio_pin pin)
{
  return HAL_GPIO_ReadPin(pin.group, pin.pin);
}


#ifdef  __cplusplus
}
#endif
#endif /* __GPIO_WRAPPER_H */
