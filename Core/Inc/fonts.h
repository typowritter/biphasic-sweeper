#ifndef __FONT_H
#define __FONT_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  const uint8_t *table;
  uint16_t width;
  uint16_t height;
} sFONT;

extern sFONT Font24x32;
extern sFONT Font16x24;
extern sFONT Font8x16;

#ifdef  __cplusplus
}
#endif
#endif /* __FONT_H */
