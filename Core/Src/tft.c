#include "tft.h"
#include "tty.h"

void tft_init()
{
  queue_reset();
}

void tft_cmd_poll()
{
  qsize_t size = queue_extract_cmd(g_cmd_buffer, CMD_MAX_SIZE);

  if (size > 0)
    ProcessMessage((PCTRL_MSG)g_cmd_buffer, size);
}

void NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state)
{
  tty_print("%d, %d, %d\r\n", screen_id, control_id, state);
}