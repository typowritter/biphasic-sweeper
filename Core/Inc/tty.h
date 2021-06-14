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

#define READSTR_MAX_BUFSIZE     100
#define STRBUF                  ewf32r9823g8398dbhq38

extern char STRBUF[READSTR_MAX_BUFSIZE];

#define tty_print(...) \
    do {\
        sprintf(STRBUF, __VA_ARGS__); \
        putstr(STRBUF);\
    } while(0)

#define tty_scan(...) \
    do {\
        readstr(STRBUF, READSTR_MAX_BUFSIZE);\
        sscanf(STRBUF, __VA_ARGS__); \
    } while(0)


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

/* -------------------------------------------------------- */

void putstr(char *ptr);
void readstr(char *ptr, size_t buflen);
void readu32(uint32_t *uint32);


#ifdef  __cplusplus
}
#endif
#endif /* __TTY_H */
