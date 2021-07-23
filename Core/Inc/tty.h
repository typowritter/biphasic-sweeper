/**
  ******************************************************************************
  * @file    tty.c
  * @author  yxnan <yxnan@pm.me>
  * @date    2020-08-26, rev. 2021-05-28
  * @brief   stm32 tty debug support
  ******************************************************************************
  * @attention
  *
  * There is no stream buffering in the hardware or HAL driver
  * Handling the string manually through USART is recommended.
  *
  ******************************************************************************
  */

#ifndef __TTY_H
#define	__TTY_H

#ifdef  __cplusplus
extern "C" {
#endif
#pragma GCC diagnostic ignored "-Wformat"

#include "usart.h"
#include <stdio.h>

#define usart_dev               huart1

/*
 * tty_print
 *  Format string and output to serial port
 */
int tty_print(char *fmt, ...);

/*
 * tty_scan
 *  Scanf from serial port
 */
int tty_scan(char *fmt, ...);

#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
    #define GETCHAR_PROTOTYPE int __io_getchar(void)
    #define PUTCHAR               __io_putchar
    #define GETCHAR               __io_getchar
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #define GETCHAR_PROTOTYPE int fgetc(FILE * f)
    #define PUTCHAR               fputc
    #define GETCHAR               fgetc
#endif  /* __GNUC__ */

PUTCHAR_PROTOTYPE;
GETCHAR_PROTOTYPE;

/* -------------------------------------------------------- */

/*
 * putstr
 *  Output a '\0'-terminated string
 */
void putstr(char *ptr);

/*
 * readstr
 *  Read a '\0'-terminated string with limited buffer size
 *  CRLF not included
 *  Input echo on
 *  Backspace ignored
 */
void readstr(char *ptr, size_t buflen);

/*
 * readu32
 *  Read a uint32 type integer, following a newline
 *  Skips non-numeric inputs
 *  Input echo on
 *  Backspace supported
 */
void readu32(uint32_t *u32);


#ifdef  __cplusplus
}
#endif
#endif /* __TTY_H */
