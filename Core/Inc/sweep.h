#ifndef __SWEEP_H
#define __SWEEP_H

#include "main.h"
#ifdef  __cplusplus
extern "C" {
#endif

#include "ad9854.h"
#include "ads124s0x.h"
#include "gpio_wrapper.h"

#define V_SRC   3.0119  /* 源电压 */
#define R_SRC   626.0   /* 直流源电阻 */
#define R_src   100.0   /* 交流源电阻 */
#define V_REF   3.0069  /* ADC参考电压 */
#define VP_2    3.4225  /* 驱动器正弦信号峰值平方 */

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
} sweep_task_t;

extern char g_type;
extern double g_mag2, g_tgp;

/* 回调函数：ADC转换完成 */
void adc_conv_complete_cb();

/* DCR测量函数 */
void dcr_measure();

void sweep_init();
void sweep_task_add(sweep_task_t task);
void sweep_task_dispatch();

void ac_esr_measure();

void ac_esr_solve(char type, double mag2, double tgp, double w,
  double esr0, double esr1, double err, int imax);

// freq_resp_t get_normal_response(char type, double vw, double esr);

double f(double mag2, double esr);
double g(double mag2, double esr);
double f1(double mag2, double tgp, double esr);
double g1(double mag2, double tgp, double esr);

#ifdef  __cplusplus
}
#endif
#endif /* __SWEEP_H */
