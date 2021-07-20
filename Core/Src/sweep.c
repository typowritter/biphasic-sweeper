#include <math.h>
#include "sweep.h"
#include "tty.h"
#include "delay.h"

#define TRANSFORM(x) (1000/(4.8212859/x - 1.6032056))

static void adc_channel_setup(int channel);
static void dds_setup(int channel);

static sweep_task_t g_task;

static struct
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

void sweep_init()
{
  g_task = TASK_IDLE;
  ads124s_update_value(ads124s_conv_mode, ads124s_mode_single);
  ads124s_update_value(ads124s_pga_en, 0);
  ads124s_update_value(ads124s_status_byte_en, 1);
  ads124s_update_value(ads124s_datarate, ads124s_datarate_x20);

  ads124s_select();
}

static void adc_channel_setup(int channel)
{
  gpio_set_low(ads124s_pin_sync);

  switch (channel)
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

static void dds_setup(int channel)
{
  switch (channel)
  {
    case N:
    case S:
      freq_convert(0); /* 关闭DDS */ break;
    case I:
    case Q:
      freq_convert(1000); break;
    default: break;
  }
}

void sweep_task_add(sweep_task_t task)
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
    dds_setup(g_adc.channel);
    adc_channel_setup(g_adc.channel);
  }
}

void sweep_task_dispatch()
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

static void task_done()
{
  g_task = TASK_IDLE;
}

/* DCR测量函数 */
void dcr_measure()
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
    task_done();
  }
}

void ac_esr_measure()
{
  static double vol_i, vol_q;
  static double sum = 0;
  static int samples = 0;

  if (samples < 10)
  {
    tty_print("%.4f\r\n", g_adc.data);
    sum += g_adc.data;
    samples++;
    return;
  }
  else if (samples == 10)
  {
    if (g_adc.channel == I)
    {
      vol_i = sum/samples;
      samples = 0;
      sum = 0;
      g_adc.channel = Q;
      adc_channel_setup(g_adc.channel);
    }
    else if (g_adc.channel == Q)
    {
      vol_q = sum/samples;
      double mag2 = vol_i*vol_i + vol_q*vol_q;
      double tgp  = vol_i/vol_q;

      tty_print("I avg: %.4f\r\n", vol_i);
      tty_print("Q avg: %.4f\r\n", vol_q);
      tty_print("\nStarting calculation...\r\n");
      tty_print("mag2: %.4f, tgp: %.4f\r\n", mag2, tgp);
      ac_esr_solve('C', mag2/VP_2, tgp, 2*M_PI*1000, 1, 10, 1e-6, 1000);

      samples = 99999;
      task_done();
    }
  }
  else
  {
    return;
  }
}

void ac_esr_solve(char type, double mag2, double tgp, double w,
  double esr0, double esr1, double err, int imax)
{
  // if (type == 'L')
  // {
  //   double (*func)(double mag2, double tgp, double esr) = g1;
  // }
  // else if (type == 'C')
  // {
  //   double (*func)(double mag2, double tgp, double esr) = f1;
  // }
  // else
  // {
  //   Error_Handler();
  //   return;
  // }

  double esr, esr2, fx0, fx1, fx2;
  int i;
  for (i = 0; i < imax; ++i)
  {
    fx0 = f1(mag2, tgp, esr0);
    fx1 = f1(mag2, tgp, esr1);
    if (fabs(fx0) < err)
    {
      esr = esr0;
      break;
    }
    else if (fabs(fx1) < err)
    {
      esr = esr1;
      break;
    }

    esr2 = esr1 - (esr1-esr0)/(fx1-fx0) * fx1;
    fx2  = f1(mag2, tgp, esr2);
    if (fabs(fx2) < err)
    {
      esr = esr2;
      break;
    }
    else
    {
      esr0 = esr1;
      esr1 = esr2;
    }
  }
  if (i >= imax)
  {
    tty_print("No ans.\r\n");
    tty_print("Fallback: %.4f\r\n", (1e9/mag2 - 1e9) / (R_src*w*R_src*w));
    return;
  }
  double v = sqrt(f(mag2, esr)) / w;
  tty_print("C: %.4f\r\n", v*1e9);
}
/*
void get_normal_response(char type, double vw, double esr)
{
  if (type == 'L')
  {
    param.mag2 = (esr*esr + vw*vw) / (vw*vw + (esr+R_src)*(esr+R_src));
    param.tgp  = (R_src * vw) / (esr*esr + R_src*esr + vw*vw);
  }
  else if (type == 'C')
  {
    param.mag2 = (esr*esr * vw*vw + 1) / (vw*vw * (esr+R_src)*(esr+R_src) + 1);
    param.tgp  = - (R_src * vw) / ((esr*esr + R_src*esr) * vw*vw + 1);
  }
  else
  {
    Error_Handler();
  }
}
*/
double f(double mag2, double esr)
{
  return (1-mag2)/(mag2*(R_src+esr)*(R_src+esr)-esr*2);
}

double g(double mag2, double esr)
{
  return 1/f(mag2, esr);
}

double f1(double mag2, double tgp, double esr)
{
  return tgp*tgp * pow(f(mag2, esr) * esr * (esr+R_src) + 1, 2)
         - R_src*R_src*f(mag2, esr);
}

double g1(double mag2, double tgp, double esr)
{
  return tgp*tgp * pow(g(mag2, esr) + esr * (esr+R_src), 2)
         - R_src*R_src*g(mag2, esr);
}