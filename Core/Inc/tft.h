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

/* 控件ID定义 */

enum {
  scr_main      = 0,
  btn_auto      = 1,
  btn_r         = 2,
  btn_l         = 3,
  btn_c         = 4,
  btn_distance  = 5,
  btn_datarate  = 6,
  btn_network   = 7,
  sld_datarate  = 11,
  txt_result    = 20,
  txt_datarate  = 21,
  gif_network   = 30,
  ico_status    = 40,
};

typedef enum {
  RECV_IDLE,
  RECV_NORMAL,
  RECV_TIMEOUT,
} recv_status_t;

/**
 * 准备好与TFT的通信
 */
void tft_init();

/**
 * 主循环中调用，检测并分派指令
 */
void tft_cmd_poll();

/**
 * UART接收回调函数
 *
 * @param status  -- RECV_NORMAL:  正常接收
 *                -- RECV_TIMEOUT: 超时接收
 */
void tft_cmd_recv_cb(recv_status_t status);

/**
 * 设置文本框显示内容，支持格式化字符串
 *
 * @param screen_id     -- 文本框所在画面ID
 * @param control_id    -- 文本框控件ID
 * @param fmt           -- 格式化字符串
 * @param [variadic]    -- 可变参数，待格式化值
 */
void tft_text_print(uint16_t screen_id, uint16_t control_id, char *fmt, ...);

#ifdef  __cplusplus
}
#endif
#endif /* __TFT_H */
