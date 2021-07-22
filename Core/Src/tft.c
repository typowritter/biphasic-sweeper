#include "tft.h"
#include "tft/conf.h"
#include "tft/cmd_process.h"
#include "tft/cmd_queue.h"
#include "tft/hmi_driver.h"
#include "tty.h"

#define RECV_BUFSIZE    CMD_MAX_SIZE

static void uart_rx_timeout_enable();

/* 双缓冲 */
static uint8_t cmd_recvbufs[2][RECV_BUFSIZE];
static __IO uint8_t curr_buf;
static __IO recv_status_t g_status;

/**
 * 准备好与TFT的通信
 */
void tft_init()
{
  queue_reset();
  uart_rx_timeout_enable();
  curr_buf = 0;
  HAL_UART_Receive_IT(&tft_dev, cmd_recvbufs[curr_buf], RECV_BUFSIZE);
}

/**
 * 主循环中调用，检测并分派指令
 */
void tft_cmd_poll()
{
  if (g_status == RECV_NORMAL)        /* 正常接收（缓冲区满），则全为有效数据 */
  {
    uint8_t *ptr = cmd_recvbufs[1-curr_buf];
    for (int i = 0; i < RECV_BUFSIZE; i++)
      queue_push(*ptr++);
  }
  else if (g_status == RECV_TIMEOUT)  /* 超时接收，则有效数据只到指令帧尾 */
  {
    uint8_t *ptr = cmd_recvbufs[1-curr_buf];
    while (queue_push(*ptr++) != CMD_EOC);
  }

  g_status = RECV_IDLE;

  qsize_t size = queue_extract_cmd(g_cmd_buffer, CMD_MAX_SIZE);

  if (size > 0)
    ProcessMessage((PCTRL_MSG)g_cmd_buffer, size);
}

/**
 * UART接收回调函数
 *
 * @param status  -- RECV_NORMAL:  正常接收
 *                -- RECV_TIMEOUT: 超时接收
 */
void tft_cmd_recv_cb(recv_status_t status)
{
  g_status = status;
  curr_buf = 1 - curr_buf;
  HAL_UART_Receive_IT(&tft_dev, cmd_recvbufs[curr_buf], RECV_BUFSIZE);
}

/* ------ 控件事件通知，覆盖 cmd_process 中的定义 ------ */

/**
 * 当按钮状态改变（或调用GetControlValue）时执行
 *
 * @param screen_id     -- 画面ID
 * @param control_id    -- 控件ID
 * @param state         -- 按钮状态：0弹起，1按下
 */
void NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
  tty_print("%d, %d, %d\r\n", screen_id, control_id, state);
}

/* ------------------ 静态函数定义 ------------------ */

static void uart_rx_timeout_enable()
{
  /* Enable receive timeout function */
  // SET_BIT(tft_dev.Instance->CR2, USART_CR2_RTOEN);
  HAL_UART_EnableReceiverTimeout(&tft_dev);
  /* Enable timeout receive interrupt */
  // SET_BIT(tft_dev.Instance->CR1, USART_CR1_RTOIE);
  __HAL_UART_ENABLE_IT(&tft_dev, UART_IT_RTO);
  /* Length of timeout required, in units of a baud duration */
  // WRITE_REG(tft_dev.Instance->RTOR, 100);
  HAL_UART_ReceiverTimeout_Config(&tft_dev, 100);
}
