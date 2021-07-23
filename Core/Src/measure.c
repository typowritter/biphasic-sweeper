#include <math.h>
#include <stdbool.h>
#include "measure.h"
#include "tty.h"
#include "delay.h"

/* -----------------------------------------
 * 各种环境常量、参数定义
 * ----------------------------------------- */
#define V_SRC         3.0119  /* 源电压 */
#define R_SRC         626.0   /* 直流源电阻 */
#define R_src         510.0   /* 交流源电阻 */
#define V_REF         3.0069  /* ADC参考电压 */
#define V_DRV         1.86    /* 驱动器正弦信号峰值 */
#define COMP_SETUP    20      /* 迟滞比较器稳定输出延时（ms） */

#define freq_conv_with_delay(freq) \
  do { \
    freq_convert(freq); \
    delay_ms(COMP_SETUP); \
  } while(0)

/* 校准参数定义 */
#define k        0.7973      /* 开路IQ两路幅频偏移 */
#define PHI0    (-2.31)      /* 开路IQ两路相频偏移 */
#define TRANSFORM(x) (1000/(4.8212859/x - 1.6032056))

/* 测量参数定义 */
#define AC_ESR_L_FREQ     100000
#define AC_ESR_C_FREQ     1000
#define SWEEP_START_FREQ  1000
#define SWEEP_STOP_FREQ   100000
#define DELTA_FREQ_MIN    200
#define DELTA_FREQ_MAX    6400
#define DELTA_PHASE_THRES 0.16
#define SWEEP_MAX_NUM     100

/* -----------------------------------------
 * 类型定义、声明
 * ----------------------------------------- */

typedef enum {
  R, L, C
} device_t;

typedef struct
{
  uint64_t freq;
  double mag;
  double phase;
} sweep_data_t;

static void adc_channel_setup();
static bool freq_visited(uint64_t freq, int upper_ix);
static void calculate_esr_z(device_t device, double vol_i, double vol_q);

#define TASK
/* 直流特性 */
static void TASK dcr_measure();
/* 1kHz相量法测电容电感 */
static void TASK ac_esr_measure(device_t device);
/* 慢开始扫频 */
static void TASK network_sweep();

/* -----------------------------------------
 * 全局变量
 * ----------------------------------------- */

static measure_task_t g_task;

static bool g_reset;

static ads124s_datarate_t
g_datarate = ads124s_datarate_x100;

static __IO struct
{
  enum {N,S,I,Q} channel;
  bool new_data;
  uint8_t status;
  double data;
} g_adc;

static sweep_data_t g_sweep_data[SWEEP_MAX_NUM];

/* -----------------------------------------
 * 函数实现
 * ----------------------------------------- */

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
  ads124s_update_value(ads124s_datarate, g_datarate);

  ads124s_select();
}

void measure_task_start(measure_task_t task)
{
  g_reset = true;
  g_task = task;

  uint32_t start_freq = 0;
  switch (g_task)
  {
    case TASK_IDLE: return;
    case TASK_SINGLE:
      g_adc.channel = S;
      gpio_set_high(dcr_switch_pin);
      start_freq = 0;
      break;
    case TASK_AC_ESR_L:
      g_adc.channel = I;
      gpio_set_low(dcr_switch_pin);
      start_freq = AC_ESR_L_FREQ;
      break;
    case TASK_AC_ESR_C:
      g_adc.channel = I;
      gpio_set_low(dcr_switch_pin);
      start_freq = AC_ESR_C_FREQ;
      break;
    case TASK_SWEEP:
      g_adc.channel = I;
      gpio_set_low(dcr_switch_pin);
      start_freq = SWEEP_START_FREQ;
      break;
    default: break;
  }
  freq_conv_with_delay(start_freq);
  adc_channel_setup();
}

void measure_task_poll()
{
  if (g_adc.new_data)
  {
    g_adc.new_data = false;
    switch (g_task)
    {
      case TASK_SINGLE: dcr_measure(); break;
      case TASK_AC_ESR_L: ac_esr_measure(L); break;
      case TASK_AC_ESR_C: ac_esr_measure(C); break;
      case TASK_SWEEP:  network_sweep(); break;
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

void measure_datarate_set(ads124s_datarate_t dr)
{
  g_datarate = dr;
}

void measure_config_update()
{
  ads124s_update_value(ads124s_datarate, g_datarate);
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

/* DCR测量函数 */
static void TASK dcr_measure()
{
  static double sum = 0;
  static int samples = 0;

  if (g_reset)
  {
    g_reset = false;
    sum = 0;
    samples = 0;
  }

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

static void TASK ac_esr_measure(device_t device)
{
  static double sum = 0;
  static int samples = 0;
  static double vol_i, vol_q;

  if (g_reset)
  {
    g_reset = false;
    sum = 0;
    samples = 0;
  }

  if (samples < 10)
  {
    sum += g_adc.data;
    samples++;
    return;
  }
  else
  {
    if (g_adc.channel == I)
    {
      vol_i = sum / samples;
      samples = 0;
      sum = 0;
      g_adc.channel = Q;
      adc_channel_setup();
    }
    else
    {
      vol_q = sum / samples;
      samples = 0;
      sum = 0;

      tty_print("I: %.4f, Q: %.4f\r\n", vol_i, vol_q);
      calculate_esr_z(device, vol_i, vol_q);
      measure_task_done();
    }
  }
}

static void TASK network_sweep()
{
  static int ix = 0;
  static uint32_t freq = SWEEP_START_FREQ;
  static uint32_t delta_freq = DELTA_FREQ_MIN;
  static double vol_i, vol_q;

  if (g_reset)
  {
    g_reset = false;
    ix = 0;
    freq = SWEEP_START_FREQ;
    delta_freq = DELTA_FREQ_MIN;
  }

  if (g_adc.channel == I)
  {
    vol_i = g_adc.data;
    g_adc.channel = Q;
    adc_channel_setup();
    return;
  }
  else
  {
    vol_q = g_adc.data;
    double mag   = k*sqrt(vol_i*vol_i + vol_q*vol_q);
    double phase = atan2(vol_q, vol_i) - PHI0;

    if (ix > 0 && fabs(phase - g_sweep_data[ix-1].phase) > DELTA_PHASE_THRES)
    {
      if (delta_freq > DELTA_FREQ_MIN)
      {
        freq -= delta_freq;
        delta_freq = DELTA_FREQ_MIN;
      }
    }
    else
    {
      delta_freq *= 2;
      if (delta_freq > DELTA_FREQ_MAX)
        delta_freq = DELTA_FREQ_MAX;
    }

    freq += delta_freq;
    g_sweep_data[ix].mag = mag;
    g_sweep_data[ix].phase = phase;
    ix++;
    tty_print("index: %d, freq: %ld, delta: %d, mag: %.4f, phase: %.4f\r\n",
      ix, freq, delta_freq, mag, phase);

    while (freq_visited(freq, ix))
    {
      tty_print("skipped %d\r\n", freq);
      freq += delta_freq;
    }

    if (ix >= 100 || freq > SWEEP_STOP_FREQ)
    {
      measure_task_done();
      return;
    }

    freq_conv_with_delay(freq);
    g_adc.channel = I;
    adc_channel_setup();
  }
}

static bool freq_visited(uint64_t freq, int upper_ix)
{
  for (int i = 0; i < upper_ix; ++i)
  {
    if (freq == g_sweep_data[i].freq)
      return true;
  }
  return false;
}

static void calculate_esr_z(device_t device, double vol_i, double vol_q)
{
  double Vo, theta, Vo_real, Vo_imag, denom, ZL_real, ZL_imag;
  double ESR, Z;

  Vo = k*sqrt(vol_i*vol_i + vol_q*vol_q);
  theta = atan2(vol_q, vol_i) - PHI0;
  Vo_real = Vo*cos(theta);
  Vo_imag = Vo*sin(theta);
  denom = (Vo_imag*Vo_imag + pow(V_DRV-Vo_real, 2));
  ZL_real = (R_src*Vo_real* (V_DRV-Vo_real) - R_src*Vo_imag*Vo_imag) / denom;
  ZL_imag = (-R_src*V_DRV*Vo_imag) / denom;

  ESR = ZL_real;

  if (device == L)
  {
    Z = fabs(1e6*ZL_imag/(2*M_PI*AC_ESR_L_FREQ));
    tty_print(
      "L: %.4f uH\r\n"
      "ESR: %.4f\r\n\n",
      Z, ESR);
  }
  else if (device == C)
  {
    Z = fabs(1e9/(ZL_imag*2*M_PI*AC_ESR_C_FREQ));
    tty_print(
      "C: %.4f nF\r\n"
      "ESR: %.4f\r\n\n",
      Z, ESR);
  }
}
