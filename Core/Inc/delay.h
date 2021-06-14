#ifndef __DELAY_H
#define __DELAY_H

#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

#include "tim.h"
#include "utils.h"

static INLINE void  delay_init();
static INLINE void  delay_us(uint32_t us);
static INLINE void  delay_ms(uint32_t ms);

/** implementation starts here */
static INLINE void
delay_init()
{
  HAL_TIM_Base_Start(&htim6);
}

static INLINE void
delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

static INLINE void
delay_us(uint32_t us)
{
  __HAL_TIM_SET_COUNTER(&htim6, 0);  // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim6) < us);  // wait for the counter to reach the us input in the parameter
}

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __DELAY_H */
