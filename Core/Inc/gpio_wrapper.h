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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

#include "gpio.h"
#include "utils.h"

typedef struct
{
  GPIO_TypeDef* port;
  uint16_t pin;
} gpio_pin;

typedef struct
{
  GPIO_TypeDef* port;
  uint8_t bits;
  uint8_t offset;
} gpio_group;

/* for fast definition */
#define DEF_GPIO(name, _port, _pin) \
  static const gpio_pin name = {.port = _port, .pin = _pin }

#define DEF_GPIO_GROUP(name, _port, _bits, _offset) \
  static const gpio_group name = {.port = _port,    \
                                  .bits = _bits,    \
                                  .offset = _offset }

/* basic GPIO functions defined as inline to save call time */
static INLINE void            gpio_set_high(gpio_pin pin);
static INLINE void            gpio_set_low(gpio_pin pin);
static INLINE void            gpio_toggle(gpio_pin pin);
static INLINE void            gpio_set(gpio_pin pin, uint8_t value);
static INLINE uint8_t         gpio_get(gpio_pin pin);
static INLINE void            gpio_set_group(gpio_group group, uint16_t value);
static INLINE uint16_t        gpio_get_group(gpio_group group);


/** implementation starts here */

static INLINE void
gpio_set_high(gpio_pin pin)
{
  HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_SET);
}

static INLINE void
gpio_set_low(gpio_pin pin)
{
  HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_RESET);
}

static INLINE void
gpio_toggle(gpio_pin pin)
{
  HAL_GPIO_TogglePin(pin.port, pin.pin);
}

static INLINE void
gpio_set(gpio_pin pin, uint8_t value)
{
  HAL_GPIO_WritePin(pin.port, pin.pin, value);
}

static INLINE uint8_t
gpio_get(gpio_pin pin)
{
  return HAL_GPIO_ReadPin(pin.port, pin.pin);
}

static INLINE void
gpio_set_group(gpio_group group, uint16_t value)
{
  const uint16_t mask = ((uint16_t)1 << group.bits) - 1;
  group.port->BSRR = ((mask & value) | ((mask & ~value) << 16)) << group.offset;
}

static INLINE uint16_t
gpio_get_group(gpio_group group)
{
  const uint16_t mask = ((uint16_t)1 << group.bits) - 1;
  return mask & (group.port->IDR >> group.offset);
}

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __GPIO_WRAPPER_H */
