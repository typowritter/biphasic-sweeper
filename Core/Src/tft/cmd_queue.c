/**
  ******************************************************************************
  * @file    cmd_queue.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-21
  * @brief   流指令队列实现
  ******************************************************************************
  */

#include "tft/conf.h"
#include "tft/cmd_queue.h"

#define CMD_HEAD 0XEE
#define CMD_TAIL 0XFFFCFFFF

static void queue_pop(qdata_t *data);
static qsize_t queue_size();

typedef struct
{
  qsize_t head;
  qsize_t tail;
  qdata_t data[QUEUE_MAX_SIZE];
} queue_t;

static queue_t g_queue = {0, 0, {0}};
static int g_cmd_count = 0;

/**
 * 清空指令数据
 */
void queue_reset()
{
  g_queue.head = g_queue.tail = 0;
  g_cmd_count = 0;
}

/**
 * UART接收的数据，通过此函数放入指令队列
 *
 * @return -- CMD_EOC 代表当前数据是指令帧尾
 */
cmd_eoc_t queue_push(qdata_t data)
{
  static uint32_t cmd_state = 0;
  qsize_t pos = (g_queue.head + 1) % QUEUE_MAX_SIZE;

  if(pos != g_queue.tail)       /* 非满状态 */
  {
    g_queue.data[g_queue.head] = data;
    g_queue.head = pos;
  }

  cmd_state = (cmd_state<<8) | data;
  if (cmd_state == CMD_TAIL)    /* 检测到指令尾 */
  {
    cmd_state = 0;
    g_cmd_count++;
    return CMD_EOC;
  }
  return CMD_NOTEOC;
}

/**
 * 从队列中取出一个数据
 */
static void queue_pop(qdata_t *data)
{
  if (g_queue.tail != g_queue.head)  /* 非空状态 */
  {
    *data = g_queue.data[g_queue.tail];
    g_queue.tail = (g_queue.tail + 1) % QUEUE_MAX_SIZE;
  }
}

static qsize_t queue_size()
{
  return (g_queue.head + QUEUE_MAX_SIZE - g_queue.tail) % QUEUE_MAX_SIZE;
}

/**
 * 从指令队列中取出一条完整的指令
 *
 * @param cmd       -- 指令接收缓存区
 * @param buf_len   -- 指令接收缓存区大小
 * @return          -- 指令长度，0表示队列中无完整指令
 */
qsize_t queue_extract_cmd(qdata_t *buffer, qsize_t buf_len)
{
  if (g_cmd_count == 0)
    return 0;

  static uint32_t cmd_state = 0;
  qsize_t cmd_pos = 0;
  qsize_t cmd_size = 0;
  qdata_t data = 0;

  while (queue_size() > 0)
  {
    queue_pop(&data);

    if (cmd_pos==0 && data!=CMD_HEAD)
      continue;

    if (cmd_pos < buf_len)              /* 防止缓冲区溢出 */
      buffer[cmd_pos++] = data;

    cmd_state = (cmd_state<<8) | data;  /* 拼接最后4个字节，组成一个32位整数 */

    //最后4个字节与帧尾匹配，得到完整帧
    if (cmd_state == CMD_TAIL)
    {
      //LED2_ON;
      cmd_size = cmd_pos;             /* 指令字节长度 */
      cmd_state = 0;                  /* 重新检测帧尾巴 */
      g_cmd_count--;

#if (CRC16_ENABLE)
      /* 去掉指令头尾EE，尾FFFCFFFF共计5个字节，只计算数据部分CRC */
      if (!CheckCRC16(buffer+1, cmd_size-5))
        return 0;

      cmd_size -= 2;                  /* 去掉CRC16（2字节） */
#endif
      return cmd_size;
    }
  }
  return 0;                           /* 没有形成完整的一帧 */
}