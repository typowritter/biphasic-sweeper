#ifndef __DELAY_H
#define __DELAY_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "tim.h"
#include "utils.h"

static INLINE void  delay_init();
void                delay_us(uint16_t us);
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

#ifdef  __cplusplus
}
#endif
#endif /* __DELAY_H */
