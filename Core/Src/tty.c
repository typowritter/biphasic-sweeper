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
#include "tty.h"

char STRBUF[READSTR_MAX_BUFSIZE];

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&usart_dev, &ch, 1, 0xFFFF);
    return ch;
}

GETCHAR_PROTOTYPE
{
    uint8_t ch;
    HAL_UART_Receive(&usart_dev, &ch, 1, 0xFFFF);
    return ch;
}

/* redirect printf and scanf
    NOTE:
    There is NO stream buffering in the hardware or HAL driver, except you implement it.
    When scanf is implemented straightforward using _read, you must always call scanf
    *before* there is any input on the RX line or you will get overrun errors.

    And don't forget to remove the stdio internal buffering. A plain call

        scanf("%d",i);

    ends up a in call to _read with a 1024 (BUFSIZ) char buffer!
    i.e. _read won't return before the UART supplied 1024 chars of input.
    This is not what you typically want for an interactive comm.
    You may remove that buffering by using

        setvbuf( stdin, NULL, _IONBF, 0 );
        scanf("%d",i);

    which results in calls to _read single char by single char.

    Anyway, I would prefer collecting a string manually using USART
    functions and then using sscanf on it.
*/

/* -------------------------------------
int _write(int file, char *ptr, int len)
{
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        __io_putchar(*ptr++);
    }
    return len;
}

int _read(int file, char *ptr, int len)
{
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        *ptr++ = __io_getchar();
    }

    return len;
}
------------------------------------- */

/*
 * putstr
 *  Output a '\0'-terminated string
 */
void putstr(char *ptr)
{
    while (*ptr != '\0')
    {
        PUTCHAR(*ptr);
        ptr++;
    }
}

/*
 * readstr
 *  Read a '\0'-terminated string with limited buffer size
 *  CRLF not included
 *  Input echo on
 *  Backspace ignored
 */
void readstr(char *ptr, size_t buflen)
{
    size_t i = 0;
    while ( i != buflen && (*ptr=GETCHAR()) != '\r' )
    {
        if (*ptr != '\b')
        {
            PUTCHAR(*ptr);
            ptr++;
            i++;
        }
    }
    PUTCHAR('\r');
    PUTCHAR('\n');
    *ptr = '\0';
}

/*
 * readu32
 *  Read a uint32 type integer, following a newline
 *  Skips non-numeric inputs
 *  Input echo on
 *  Backspace supported
 */
void readu32(uint32_t *uint32)
{
    char ch;
    uint32_t res = 0;
    size_t valid_cnt = 0;

    while((ch=GETCHAR()) != '\r')
    {
        if (ch >= '0' && ch <= '9')
        {
            res = res*10 + (ch - '0');
            PUTCHAR(ch);
            valid_cnt++;
        }
        else if (ch == '\b' && valid_cnt > 0)
        {
            res /= 10;
            PUTCHAR('\b');
            PUTCHAR(' ');
            PUTCHAR('\b');
            valid_cnt--;
        }
    }
    *uint32 = res;
}
