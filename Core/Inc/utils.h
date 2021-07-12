/**
  ******************************************************************************
  * @file    utils.h
  * @author  yxnan <yxnan@pm.me>
  * @date    2021-05-21
  * @brief   utilities
  ******************************************************************************
  */

#ifndef __UTILS_H
#define __UTILS_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define INLINE __attribute__((always_inline)) inline

#define min(a, b) ((a) < (b) ? (a) : (b))

#define _str(s) #s
#define str(s) _str(s)

#define undefined Error_Handler
#define todo      Error_Handler

/*
 * compound literals, supported since C99
 * see: 'https://en.cppreference.com/w/c/language/compound_literal'
 */
#define lit2addr(lit) (&(uint8_t){lit})

#define byte2bin_pattern "%c%c%c%c%c%c%c%c\n"
#define byte2bin(byte)  \
  (byte & (1<<7) ? '1' : '0'), \
  (byte & (1<<6) ? '1' : '0'), \
  (byte & (1<<5) ? '1' : '0'), \
  (byte & (1<<4) ? '1' : '0'), \
  (byte & (1<<3) ? '1' : '0'), \
  (byte & (1<<2) ? '1' : '0'), \
  (byte & (1<<1) ? '1' : '0'), \
  (byte & (1<<0) ? '1' : '0')


#define DEBUG_GEN(fn, prefix, ...) fn(prefix __VA_ARGS__)

/* bool */
typedef uint8_t bool;
#ifndef false
#define false 0
#endif
#ifndef true
#define true (!false)
#endif

#ifdef  __cplusplus
}
#endif

#endif /* __UTILS_H */
