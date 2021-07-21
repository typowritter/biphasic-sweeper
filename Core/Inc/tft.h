/**
  ******************************************************************************
  * @file    tft.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-21
  * @brief   大彩TFT组态屏顶层模块
  ******************************************************************************
  */

#ifndef __TFT_H
#define __TFT_H

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * 准备好与TFT的通信
 */
void tft_init();

/**
 * 主循环中调用，检测并分派指令
 */
void tft_cmd_poll();

/**
 * UART接收回调函数，将数据放入接收队列
 */
void tft_cmd_recv_cb();

#ifdef  __cplusplus
}
#endif
#endif /* __TFT_H */
