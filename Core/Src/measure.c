#include <math.h>
#include "measure.h"
#include "tty.h"
#include "delay.h"


/* 各种环境常量 */
#define V_SRC         3.0119  /* 源电压 */
#define R_SRC         626.0   /* 直流源电阻 */
#define R_src         100.0   /* 交流源电阻 */
#define V_REF         3.0069  /* ADC参考电压 */
#define VP_2          3.4225  /* 驱动器正弦信号峰值平方 */
#define COMP_SETUP    20      /* 迟滞比较器稳定输出延时（ms） */

#define freq_conv_with_delay(freq) \
  do { \
    freq_convert(freq); \
    delay_ms(COMP_SETUP); \
  } while(0)

/* 校准参数定义 */
#define PHI0    0.7280  /* 开路IQ两路相频偏移 */
#define TRANSFORM(x) (1000/(4.8212859/x - 1.6032056))

#define TASK
static void adc_channel_setup();
static void dds_setup();
/* 直流特性 */
static void TASK dcr_measure();
/* 1kHz扫频测电容电感 */
static void TASK ac_esr_measure();

static measure_task_t g_task;

static __IO struct
{
  enum {N,S,I,Q} channel;
  bool new_data;
  uint8_t status;
  double data;
} g_adc;

/* 回调函数：ADC转换完成 */
void adc_conv_complete_cb()
{
  ads124s_conv_result_t res = ads124s_read_conv_data();
  g_adc.data = (double)res.data*V_REF/(1<<23);
  g_adc.status = res.status;
  g_adc.new_data = true;
}

void measure_init()
{
  g_task = TASK_IDLE;
  ads124s_update_value(ads124s_conv_mode, ads124s_mode_single);
  ads124s_update_value(ads124s_pga_en, 0);
  ads124s_update_value(ads124s_status_byte_en, 1);
  ads124s_update_value(ads124s_datarate, ads124s_datarate_x20);

  ads124s_select();
}

void measure_task_start(measure_task_t task)
{
  if (g_task != task)
  {
    g_task = task;
    switch (g_task)
    {
      case TASK_SINGLE:
        g_adc.channel = S;
        gpio_set_high(dcr_switch_pin);
        break;
      case TASK_AC_ESR:
        g_adc.channel = I;
        gpio_set_low(dcr_switch_pin);
        break;
      default: break;
    }
    dds_setup();
    adc_channel_setup();
  }
}

void measure_task_poll()
{
  if (g_adc.new_data)
  {
    g_adc.new_data = false;
    switch (g_task)
    {
      case TASK_SINGLE: dcr_measure(); break;
      case TASK_AC_ESR: ac_esr_measure(); break;
      default: break;
    }

    gpio_set_low(ads124s_pin_sync);
    if (g_task != TASK_IDLE)
    {
      delay_us(1);
      gpio_set_high(ads124s_pin_sync);
    }
  }
}

void measure_task_done()
{
  g_task = TASK_IDLE;
}

/* ------------- 静态函数定义 ------------------ */

static void adc_channel_setup()
{
  gpio_set_low(ads124s_pin_sync);

  switch (g_adc.channel)
  {
    case S:
      ads124s_set_channel(DCR_CHAN_P, DCR_CHAN_N);
      gpio_set_high(ads124s_pin_sync);
      break;

    case I:
      ads124s_set_channel(IDAC_CHAN_P, IDAC_CHAN_N);
      gpio_set_high(ads124s_pin_sync);
      break;

    case Q:
      ads124s_set_channel(QDAC_CHAN_P, QDAC_CHAN_N);
      gpio_set_high(ads124s_pin_sync);
      break;

    default: break;
  }
}

static void dds_setup()
{
  switch (g_adc.channel)
  {
    case N:
    case S:
      freq_convert(0); /* 关闭DDS */ break;
    case I:
    case Q:
      break;
    default: break;
  }
}

/* DCR测量函数 */
static void TASK dcr_measure()
{
  static double sum = 0;
  static int samples = 0;
  if (samples < 10)
  {
    tty_print(
      "STATUS: 0x%X\r\n"
      "Conv: %.4f\r\n"
      "DCR: %.4f\r\n\n",
      g_adc.status, g_adc.data,
      TRANSFORM(g_adc.data));
      // vol*626/(3.0119-vol));

    sum += g_adc.data;
    samples++;
    return;
  }
  else
  {
    tty_print("AVG: %.4f\r\n\n", sum/samples);
    measure_task_done();
  }
}

static void TASK ac_esr_measure()
{
  static bool first_dropped = false;
  static uint64_t freq = 1000;
  static uint64_t d_freq = 1000;
  static double vol_i;
  static double vol_q;

  if (!first_dropped)
  {
    first_dropped = true;
    freq_conv_with_delay(freq);
    return;
  }
  else
  {
    if (g_adc.channel == I)
    {
      vol_i = g_adc.data;
      g_adc.channel = Q;
      adc_channel_setup();
    }
    else
    {
      vol_q = g_adc.data;
      double phase = tan(atan(vol_i/vol_q) - PHI0);
      tty_print("%.4f, ", phase);

      if (freq % 10000 == 0)
        tty_print("\r\n");

      if (freq < 100000)
      {
        freq += d_freq;
        g_adc.channel = I;
        freq_conv_with_delay(freq);
        adc_channel_setup();
      }
      else
      {
        measure_task_done();
      }
    }
  }
}
