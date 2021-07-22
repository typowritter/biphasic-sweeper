/**
  ******************************************************************************
  * @file    cmd_queue.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-07-21
  * @brief   流指令队列实现
  ******************************************************************************
  */

#ifndef __CMD_QUEUE
#define __CMD_QUEUE

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef uint8_t  qdata_t;
typedef uint16_t qsize_t;

typedef enum {
  CMD_EOC,    /* End Of Command */
  CMD_NOTEOC,
} cmd_eoc_t;

/**
 * 清空指令数据
 */
void queue_reset(void);

/**
 * UART接收的数据，通过此函数放入指令队列
 *
 * @return -- CMD_EOC 代表当前数据是指令帧尾
 */
cmd_eoc_t queue_push(qdata_t data);

/**
 * 从指令队列中取出一条完整的指令
 *
 * @param cmd       -- 指令接收缓存区
 * @param buf_len   -- 指令接收缓存区大小
 * @return          -- 指令长度，0表示队列中无完整指令
 */
qsize_t queue_extract_cmd(qdata_t *cmd, qsize_t buf_len);

#ifdef  __cplusplus
}
#endif
#endif /* __CMD_QUEUE */
