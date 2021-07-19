#ifndef __SWEEP_H
#define __SWEEP_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "ad9854.h"
#include "ads124s0x.h"

#define V_SRC   3.0119  /* 源电压 */
#define R_SRC   626.0   /* 源电阻 */
#define V_REF   3.0069  /* ADC参考电压 */

#define DCR_CHAN_P      ads124s_chan_ain4   /* DCR测量输入端 */
#define DCR_CHAN_N      ads124s_chan_ain2   /* DCR测量接地端 */
#define IDAC_CHAN_P     ads124s_chan_ain0   /* IDAC差分输入端 */
#define IDAC_CHAN_N     ads124s_chan_ain1   /* IDAC差分输入端 */
#define QDAC_CHAN_P     ads124s_chan_ain8   /* QDAC差分输入端 */
#define QDAC_CHAN_N     ads124s_chan_ain9   /* QDAC差分输入端 */

/* 回调函数：ADC转换完成 */
void adc_conv_complete_cb();

/* DCR测量函数 */
void dcr_measure();

#ifdef  __cplusplus
}
#endif
#endif /* __SWEEP_H */
