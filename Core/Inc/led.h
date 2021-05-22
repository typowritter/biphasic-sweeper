#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  LED_R,
  LED_G,
  LED_B,
  LED_OFF,
} led_color;

void LED_SetColor(led_color color);

#ifdef __cplusplus
}
#endif
#endif /* __LED_H */
