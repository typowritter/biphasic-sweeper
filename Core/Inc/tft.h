#ifndef __TFT_H
#define __TFT_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "tft/cmd_process.h"
#include "tft/cmd_queue.h"
#include "tft/hmi_driver.h"

void tft_init();
void tft_cmd_poll();

#ifdef  __cplusplus
}
#endif
#endif /* __TFT_H */
