#ifndef __measure_H
#define __measure_H

#include "main.h"
#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-const-variable"

#include "ad9854.h"
#include "ads124s0x.h"
#include "gpio_wrapper.h"

#define DCR_CHAN_P      ads124s_chan_ain4   /* DCR测量输入端 */
#define DCR_CHAN_N      ads124s_chan_ain0   /* DCR测量接地端 */
#define IDAC_CHAN_P     ads124s_chan_ain1   /* IDAC差分输入端 */
#define IDAC_CHAN_N     ads124s_chan_ain2   /* IDAC差分输入端 */
#define QDAC_CHAN_P     ads124s_chan_ain11  /* QDAC差分输入端 */
#define QDAC_CHAN_N     ads124s_chan_ain10  /* QDAC差分输入端 */

DEF_GPIO(dcr_switch_pin, DCR_SWITCH_GPIO_Port, DCR_SWITCH_Pin);

typedef enum {
  TASK_IDLE,
  TASK_SINGLE,
  TASK_AC_ESR,
} measure_task_t;

extern char g_type;
extern double g_mag2, g_tgp;

/* 回调函数：ADC转换完成 */
void adc_conv_complete_cb();

void measure_init();
void measure_task_start(measure_task_t task);
void measure_task_poll();
void measure_task_done();

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __measure_H */
