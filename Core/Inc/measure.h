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
  TASK_SWEEP,
} measure_task_t;

/**
 * 外部中断回调函数，ADC转换完成
 */
void adc_conv_complete_cb();

/**
 * 准备好ADC和任务状态
 */
void measure_init();

/**
 * 开始一个测量任务，不需要考虑当前任务是否完成
 */
void measure_task_start(measure_task_t task);

/**
 * 主循环中调用，检测并执行测量任务
 */
void measure_task_poll();

/**
 * 结束当前测量任务
 */
void measure_task_done();

#pragma GCC diagnostic pop
#ifdef  __cplusplus
}
#endif
#endif /* __measure_H */
