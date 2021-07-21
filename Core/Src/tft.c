#include "tft.h"
#include "tft/cmd_process.h"
#include "tft/cmd_queue.h"
#include "tft/hmi_driver.h"
#include "tty.h"

static uint8_t tft_cmd_recvdata;

void tft_init()
{
  queue_reset();
  HAL_UART_Receive_IT(&tft_dev, &tft_cmd_recvdata, 1);  /* 开始接收数据 */
}

void tft_cmd_poll()
{
  qsize_t size = queue_extract_cmd(g_cmd_buffer, CMD_MAX_SIZE);

  if (size > 0)
    ProcessMessage((PCTRL_MSG)g_cmd_buffer, size);
}

void tft_cmd_recv_cb()
{
  queue_push(tft_cmd_recvdata);
  HAL_UART_Receive_IT(&tft_dev, &tft_cmd_recvdata, 1);  /* 准备接收下一个数据 */
}

/*!
*  \brief  按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*/
void NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
  tty_print("%d, %d, %d\r\n", screen_id, control_id, state);
}
