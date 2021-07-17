#include "sweep.h"
#include "tty.h"

#define TRANSFORM(x) (1000/(4.8212859/x - 1.6032056))

/* 回调函数：ADC转换完成 */
void adc_conv_complete_cb()
{
  dcr_measure();
}

/* DCR测量函数 */
void dcr_measure()
{
  ads124s_conv_result_t res = ads124s_read_conv_data();
  double vol = (double)res.data*3.0069/(1<<23);
  tty_print(
    "STATUS: 0x%X\r\n"
    "Conv: 0x%X, %.4f\r\n"
    "DCR: %.4f\r\n\n",
    res.status, res.data, vol,
    TRANSFORM(vol));
    // vol*626/(3.0119-vol));
}